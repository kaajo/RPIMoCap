/*
 * This file is part of the RPIMoCap (https://github.com/kaajo/RPIMoCap).
 * Copyright (c) 2019 Miroslav Krajicek.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "RPIMoCap/Server/wandcalibration.h"

#include <QThread>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/core/eigen.hpp>
#include <ceres/ceres.h>

#include <limits>
#include <iostream>
#include <queue>

class ReprojectionError {
public:
    static ceres::CostFunction* Create(cv::Point2f observation, cv::Mat cameraMatrix,
                                       float realDistance1, cv::Point3d &nextPoint1,
                                       float realDistance2, cv::Point3d &nextPoint2) {
        return (new ceres::NumericDiffCostFunction<ReprojectionError, ceres::CENTRAL, 3, 6, 3>(
            new ReprojectionError(observation, cameraMatrix, realDistance1, nextPoint1, realDistance2, nextPoint2)));
    }

    bool operator()(const double* const camera, const double* const point, double* residuals) const
    {
        std::vector<cv::Point3f> triangulatedPtsFloat(1, cv::Point3f(point[0], point[1], point[2]));

        const cv::Vec3f rVec(camera[0], camera[1], camera[2]);
        const cv::Vec3f tVec(camera[3], camera[4], camera[5]);

        std::vector<cv::Point2f> projPixels;
        cv::projectPoints(triangulatedPtsFloat, rVec, tVec, m_cameraMatrix, cv::noArray(), projPixels);

        const cv::Point2f res = projPixels.front() - m_observation;

        residuals[0] = res.x;
        residuals[1] = res.y;

        cv::Point3d triangulatedPoint(point[0], point[1], point[2]);

        residuals[2] = std::abs(cv::norm(triangulatedPoint - m_nextPoint1) - m_realDistance1) +
                       std::abs(cv::norm(triangulatedPoint - m_nextPoint2) - m_realDistance2);

        return true;
    }

private:
    ReprojectionError(cv::Point2f observation, cv::Mat cameraMatrix,
                      float realDistance1, cv::Point3d &nextPoint1,
                      float realDistance2, cv::Point3d &nextPoint2)
        : m_observation(observation)
        , m_cameraMatrix(cameraMatrix)
        , m_realDistance1(realDistance1)
        , m_nextPoint1(nextPoint1)
        , m_realDistance2(realDistance2)
        , m_nextPoint2(nextPoint2) {}

    cv::Point2f m_observation;
    cv::Mat m_cameraMatrix;

    float m_realDistance1;
    cv::Point3d &m_nextPoint1;

    float m_realDistance2;
    cv::Point3d &m_nextPoint2;
};

WandCalibration::WandCalibration(QMap<int, std::shared_ptr<CameraSettings> > &cameraSettings,
                                 RPIMoCap::CameraParams camData, QObject *parent)
    : QObject(parent)
    , m_cameraSettings(cameraSettings)
    , m_camData(camData)
{
    m_wandPoints.push_back({-25.0,0.0,0.0});
    m_wandPoints.push_back({10.0,0.0,0.0});
    m_wandPoints.push_back({25.0,0.0,0.0});
}

void WandCalibration::addFrame(const QMap<int, std::vector<cv::Point2f> > &points)
{
    if (finished) return;

    std::vector<std::pair<int, std::vector<cv::Point2f>>> detectedPoints;

    //detect points (order)
    for (auto &camID : points.keys())
    {
        if (auto detPts = detect3pWand(points[camID]))
        {
            detectedPoints.push_back({camID, detPts.value()});
        }
    }

    //add detected points to observation
    for (size_t i = 0; i < detectedPoints.size(); ++i)
    {
        for (size_t j = i + 1; j < detectedPoints.size(); ++j)
        {
            ObsDetection &obsDet = m_observedDetections[{detectedPoints[i].first,detectedPoints[j].first}];

            float maxDiff = 0.0;

            if (!obsDet.firstPixels.empty())
            {
                for (int wandIndex = 0; wandIndex < m_wandPoints.size(); ++wandIndex)
                {
                    const float diff = cv::norm(detectedPoints[i].second[wandIndex] - obsDet.firstPixels[obsDet.firstPixels.size() - m_wandPoints.size() + wandIndex]);
                    const float diff2 = cv::norm(detectedPoints[j].second[wandIndex] - obsDet.secondPixels[obsDet.secondPixels.size() - m_wandPoints.size() + wandIndex]);
                    if (std::max(diff, diff2) > maxDiff)
                    {
                        maxDiff = std::max(diff, diff2);
                    }
                }
            }

            if (maxDiff > 5.0 || obsDet.firstPixels.empty() || obsDet.secondPixels.empty())
            {
                obsDet.firstPixels.insert(obsDet.firstPixels.end(), detectedPoints[i].second.begin(), detectedPoints[i].second.end());
                obsDet.secondPixels.insert(obsDet.secondPixels.end(), detectedPoints[j].second.begin(), detectedPoints[j].second.end());

                std::cout << "add observation:" << obsDet.firstPixels.size()/m_wandPoints.size() << std::endl;
            }
        }
    }

    for (auto detIt = m_observedDetections.begin(); detIt != m_observedDetections.end(); ++detIt)
    {
        const int detectedFrames = std::min(detIt->firstPixels.size(), detIt->secondPixels.size());

        if (detectedFrames < 100 * m_wandPoints.size())
        {
            //skip cameras with small number of samples
            continue;
        }

        const cv::Mat essential = cv::findEssentialMat(detIt->firstPixels, detIt->secondPixels, m_camData.cameraMatrix, cv::RANSAC, 0.999, 0.1);

        cv::Mat rotation, translation;
        cv::recoverPose(essential, detIt->firstPixels, detIt->secondPixels, m_camData.cameraMatrix, rotation, translation);

        cv::Mat transformMatrixSecond;
        cv::hconcat(rotation, translation, transformMatrixSecond);

        const cv::Mat projectionMatFirst = m_camData.cameraMatrix * cv::Mat::eye(3,4, CV_64FC1);
        const cv::Mat projectionMatSecond = m_camData.cameraMatrix * transformMatrixSecond;

        cv::Mat triangulatedPoints;
        cv::triangulatePoints(projectionMatFirst, projectionMatSecond, detIt->firstPixels, detIt->secondPixels, triangulatedPoints);

        std::vector<cv::Point3d> triangulatedPoints3D;

        for (size_t i = 0; i < triangulatedPoints.cols; ++i)
        {
            const float w = triangulatedPoints.at<float>(3,i);
            triangulatedPoints3D.push_back({triangulatedPoints.at<float>(0,i)/w,
                                            triangulatedPoints.at<float>(1,i)/w,
                                            triangulatedPoints.at<float>(2,i)/w});
        }

        const float scale = computeScale(transformMatrixSecond, triangulatedPoints3D);

        //scale scene
        for(auto &triangulatedPoint : triangulatedPoints3D)
        {
            triangulatedPoint *= scale;
        }
        translation *= scale;

        const Eigen::Affine3f estimatedTransform(cv::Affine3d(rotation, translation).cast<float>());

        //compute error and update transform
        const float errorFirst = computeReprojectionError(detIt->firstPixels, triangulatedPoints3D, Eigen::Affine3f::Identity());
        const float errorSecond = computeReprojectionError(detIt->secondPixels, triangulatedPoints3D, estimatedTransform);

        const float error = (errorFirst + errorSecond)/2.0f;

        std::cout << "error: " << detIt.key().first << " " << errorFirst << std::endl;
        std::cout << "error: " << detIt.key().second << " " << errorSecond << std::endl;

        if (error < detIt->reprojectionError)
        {
            detIt->reprojectionError = error;
            detIt->transform = estimatedTransform;
            detIt->triangulatedPoints = triangulatedPoints3D;

            m_cameraSettings[detIt.key().second]->setTransform(estimatedTransform);
        }
    }

    //check if we can go to another stage of calibration
    auto cameraIDs = m_cameraSettings.keys();

    QVector<int> discovered;
    std::queue<int> pointQueue;

    //find first good node and mark it as reference camera
    for (auto detIt = m_observedDetections.begin(); detIt != m_observedDetections.end(); ++detIt)
    {
        if (detIt->reprojectionError < m_errorTreshold)
        {
            discovered.push_back(detIt.key().first);
            pointQueue.push(detIt.key().first);
            break;
        }
    }

    //search
    while (!pointQueue.empty())
    {
        const int currentNode = pointQueue.front();
        pointQueue.pop();

        for (auto detIt = m_observedDetections.begin(); detIt != m_observedDetections.end(); ++detIt)
        {
            if (detIt.key().first == currentNode && detIt->reprojectionError < m_errorTreshold)
            {
                if (!discovered.contains(detIt.key().second))
                {
                    pointQueue.push(detIt.key().second);
                    discovered.push_back(detIt.key().second);
                }
            }

            if (detIt.key().second == currentNode && detIt->reprojectionError < m_errorTreshold)
            {
                if (!discovered.contains(detIt.key().first))
                {
                    pointQueue.push(detIt.key().first);
                    discovered.push_back(detIt.key().first);
                }
            }
        }
    }

    if (discovered.size() < m_cameraSettings.size())
    {
        return;
    }

    for (auto &disc : discovered)
    {
        std::cout << "transform for camera " << disc << " ready for next stage" << std::endl;
    }

    //bundle adjustment stage
    //TODO for more than 2 cams;
    auto &data = m_observedDetections.first();

    const cv::Mat projectionMatFirst = m_camData.cameraMatrix * cv::Mat::eye(3,4, CV_64FC1);

    double* dataCamFirst = data.cameraFirstParams(m_camData.cameraMatrix.at<double>(0,0));
    double* dataCamSecond = data.cameraSecondParams(m_camData.cameraMatrix.at<double>(0,0));

    float leftToMiddleDist = cv::norm(m_wandPoints[1] - m_wandPoints[0]);
    float leftToRightDist = cv::norm(m_wandPoints[2] - m_wandPoints[0]);
    float middleToRightDist = cv::norm(m_wandPoints[2] - m_wandPoints[1]);

    ceres::Problem problem;
    for (int i = 0; i < data.firstPixels.size(); i = i + m_wandPoints.size()) {

        {
        ceres::CostFunction* costLeftFirstFunction =
            ReprojectionError::Create(data.firstPixels[i], m_camData.cameraMatrix,
                                      leftToMiddleDist, data.triangulatedPoints[i+1],
                                      leftToRightDist, data.triangulatedPoints[i+2]);

        problem.AddResidualBlock(costLeftFirstFunction, nullptr, dataCamFirst,
                                 &data.triangulatedPoints[i].x);

        ceres::CostFunction* costLeftSecondFunction =
            ReprojectionError::Create(data.secondPixels[i], m_camData.cameraMatrix,
                                      leftToMiddleDist, data.triangulatedPoints[i+1],
                                      leftToRightDist, data.triangulatedPoints[i+2]);

        problem.AddResidualBlock(costLeftSecondFunction, nullptr, dataCamSecond,
                                 &data.triangulatedPoints[i].x);
        }

        {
            ceres::CostFunction* costMiddleFirstFunction =
                ReprojectionError::Create(data.firstPixels[i+1], m_camData.cameraMatrix,
                                          leftToMiddleDist, data.triangulatedPoints[i],
                                          middleToRightDist, data.triangulatedPoints[i+2]);

            problem.AddResidualBlock(costMiddleFirstFunction, nullptr, dataCamFirst,
                                     &data.triangulatedPoints[i+1].x);

            ceres::CostFunction* costMiddleSecondFunction =
                ReprojectionError::Create(data.secondPixels[i+1], m_camData.cameraMatrix,
                                          leftToMiddleDist, data.triangulatedPoints[i],
                                          middleToRightDist, data.triangulatedPoints[i+2]);

            problem.AddResidualBlock(costMiddleSecondFunction, nullptr, dataCamSecond,
                                     &data.triangulatedPoints[i+1].x);
        }

        {
            ceres::CostFunction* costRightFirstFunction =
                ReprojectionError::Create(data.firstPixels[i+2], m_camData.cameraMatrix,
                                          leftToRightDist, data.triangulatedPoints[i],
                                          middleToRightDist, data.triangulatedPoints[i+1]);

            problem.AddResidualBlock(costRightFirstFunction, nullptr, dataCamFirst,
                                     &data.triangulatedPoints[i+2].x);

            ceres::CostFunction* costRightSecondFunction =
                ReprojectionError::Create(data.secondPixels[i+2], m_camData.cameraMatrix,
                                          leftToRightDist, data.triangulatedPoints[i],
                                          middleToRightDist, data.triangulatedPoints[i+1]);

            problem.AddResidualBlock(costRightSecondFunction, nullptr, dataCamSecond,
                                     &data.triangulatedPoints[i+2].x);
        }


    }

    double minRot = -M_PI;
    double maxRot = M_PI;

    problem.SetParameterLowerBound(dataCamFirst, 0, minRot);
    problem.SetParameterUpperBound(dataCamFirst, 0, maxRot);
    problem.SetParameterLowerBound(dataCamFirst, 1, minRot);
    problem.SetParameterUpperBound(dataCamFirst, 1, maxRot);
    problem.SetParameterLowerBound(dataCamFirst, 2, minRot);
    problem.SetParameterUpperBound(dataCamFirst, 2, maxRot);

    problem.SetParameterLowerBound(dataCamSecond, 0, minRot);
    problem.SetParameterUpperBound(dataCamSecond, 0, maxRot);
    problem.SetParameterLowerBound(dataCamSecond, 1, minRot);
    problem.SetParameterUpperBound(dataCamSecond, 1, maxRot);
    problem.SetParameterLowerBound(dataCamSecond, 2, minRot);
    problem.SetParameterUpperBound(dataCamSecond, 2, maxRot);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = QThread::idealThreadCount();

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    std::cout << "ROTATION AFTER: " << data.cameraFirstData[0] * 180.0/M_PI << " "
              << data.cameraFirstData[1] * 180.0/M_PI << " "
              << data.cameraFirstData[2] * 180.0/M_PI << std::endl;
    std::cout << "TRANSLATION AFTER: " << data.cameraFirstData[3] << " "
              << data.cameraFirstData[4] << " " << data.cameraFirstData[5] << std::endl;

    std::cout << "ROTATION AFTER: " << data.cameraSecondData[0] * 180.0/M_PI << " "
              << data.cameraSecondData[1] * 180.0/M_PI << " "
              << data.cameraSecondData[2] * 180.0/M_PI << std::endl;
    std::cout << "TRANSLATION AFTER: " << data.cameraSecondData[3] << " "
              << data.cameraSecondData[4] << " " << data.cameraSecondData[5] << std::endl;

    finished = true;
}

std::optional<std::vector<cv::Point2f>> WandCalibration::detect4pWand(const std::vector<cv::Point2f> &pts)
{
    if (pts.size() != 4)
    {
        return std::nullopt;
    }

    cv::Mat distMap(pts.size(), pts.size(), CV_32FC1,
                    cv::Scalar(std::numeric_limits<float>::infinity()));

    for (size_t i = 0; i < pts.size(); ++i)
    {
        for (size_t j = i+1; j < pts.size(); ++j)
        {
            distMap.at<float>(i,j) = distMap.at<float>(j,i) = cv::norm(pts[i] - pts[j]);
        }
    }

    double min = std::numeric_limits<double>::lowest();
    double max = std::numeric_limits<double>::max();
    cv::Point minLoc(-1, -1);
    cv::Point maxLoc(-1, -1);
    cv::Mat diagMask = cv::Scalar::all(1) - cv::Mat::eye(pts.size(), pts.size(), CV_8UC1);
    cv::minMaxLoc(distMap, &min, &max, &minLoc, &maxLoc, diagMask);

    size_t borderL = maxLoc.x;
    size_t borderR = maxLoc.y;
    size_t middle = minLoc.x;
    size_t cross = minLoc.y;

    if (distMap.at<float>(cross,borderL) <  distMap.at<float>(middle,borderL))
    {
        std::swap(cross, middle);
    }

    if (distMap.at<float>(middle,borderL) <  distMap.at<float>(middle,borderR))
    {
        std::swap(borderL, borderR);
    }

    return std::vector<cv::Point2f>{pts[borderL], pts[middle], pts[borderR], pts[cross]};
}

std::optional<std::vector<cv::Point2f> > WandCalibration::detect3pWand(const std::vector<cv::Point2f> &pts)
{
    if (pts.size() != 3)
    {
        return std::nullopt;
    }

    std::vector<float> distances;
    distances.push_back(cv::norm(pts[1] - pts[0]) + cv::norm(pts[2] - pts[0]));
    distances.push_back(cv::norm(pts[0] - pts[1]) + cv::norm(pts[2] - pts[1]));
    distances.push_back(cv::norm(pts[0] - pts[2]) + cv::norm(pts[1] - pts[2]));

    //TODO take also wand points for correct order
    const size_t middleID = (std::min_element(distances.begin(), distances.end()) - distances.begin());
    const size_t leftID = (std::max_element(distances.begin(), distances.end()) - distances.begin());

    QSet<size_t> indexes{0, 1, 2};
    indexes.remove(middleID);
    indexes.remove(leftID);
    const size_t rightID = indexes.values().first();

    return std::vector<cv::Point2f>{pts[leftID], pts[middleID], pts[rightID]};
}

float WandCalibration::computeReprojectionError(const std::vector<cv::Point2f> &pixels, const std::vector<cv::Point3d> &triangulatedPoints,
                                                const Eigen::Affine3f &estTransform)
{
    //TODO remove
    std::vector<cv::Point3f> triangulatedPtsFloat;

    for (auto &pt : triangulatedPoints)
    {
        triangulatedPtsFloat.push_back(cv::Point3f(pt.x, pt.y, pt.z));
    }

    cv::Affine3f cvEstTransform(estTransform);

    cv::Mat rVec;
    cv::Rodrigues(cvEstTransform.rotation(), rVec);

    cv::Vec3f tVec(estTransform.translation().x(),estTransform.translation().y(),estTransform.translation().z());

    std::vector<cv::Point2f> projPixels;
    cv::projectPoints(triangulatedPtsFloat, rVec, tVec, m_camData.cameraMatrix, m_camData.distortionCoeffs, projPixels);
    assert(projPixels.size() == pixels.size());

    float sum = 0.0f;
    for (size_t i = 0; i < pixels.size(); ++i)
    {
        sum += cv::norm(projPixels[i] - pixels[i]);
    }

    return sum/pixels.size();
}

float WandCalibration::computeScale(const cv::Mat &transformEstimation, std::vector<cv::Point3d> &triangulatedPoints)
{
    assert(triangulatedPoints.size() % m_wandPoints.size() == 0);

    float scale = 0.0f;
    const int numOfObservations = triangulatedPoints.size()/m_wandPoints.size();
    for (size_t i = 1; i < m_wandPoints.size(); ++i)
    {
        const float realDistance = cv::norm(m_wandPoints[i] - m_wandPoints[i - 1]);

        float sum = 0.0f;
        for (size_t pntI = 0; pntI < triangulatedPoints.size(); pntI = pntI + m_wandPoints.size())
        {
            sum += cv::norm(triangulatedPoints[pntI + i] - triangulatedPoints[pntI + i - 1]);
        }
        const float averageVirtualDistance = sum/static_cast<float>(numOfObservations);
        scale += realDistance/averageVirtualDistance;
    }

    return scale/static_cast<float>(m_wandPoints.size() - 1);
}
