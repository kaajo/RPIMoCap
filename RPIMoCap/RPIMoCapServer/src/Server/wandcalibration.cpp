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

#include <limits>
#include <iostream>
#include <queue>

namespace RPIMoCap {

WandCalibration::WandCalibration(QMap<QUuid, std::shared_ptr<CameraSettings> > &cameraSettings,
                                 Camera::Intrinsics camData, QObject *parent)
    : QObject(parent)
    , m_cameraSettings(cameraSettings)
    , m_camData(camData)
{
    m_wandPoints.push_back({-25.0,0.0,0.0});
    m_wandPoints.push_back({10.0,0.0,0.0});
    m_wandPoints.push_back({25.0,0.0,0.0});
}

void WandCalibration::addFrame(const QMap<QUuid, std::vector<cv::Point2f> > &points)
{
    if (finished) return;

    std::vector<std::pair<QUuid, std::vector<cv::Point2f>>> detectedPoints;

    //detect points (order)
    for (auto &camID : points.keys())
    {
        if (auto detPts = WandDetector::detect3pWand(points[camID]))
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
        cv::Mat cameraMatrix;
        m_camData.cameraMatrix.convertTo(cameraMatrix, CV_64FC1);

        const int detectedFrames = std::min(detIt->firstPixels.size(), detIt->secondPixels.size());

        if (detectedFrames < 300 * m_wandPoints.size())
        {
            //skip cameras with small number of samples
            continue;
        }

        const cv::Mat essential = cv::findEssentialMat(detIt->firstPixels, detIt->secondPixels, cameraMatrix, cv::RANSAC, 0.999, 0.1);

        cv::Mat rotation, translation;
        cv::recoverPose(essential, detIt->firstPixels, detIt->secondPixels, cameraMatrix, rotation, translation);

        cv::Mat transformMatrixSecond;
        cv::hconcat(rotation, translation, transformMatrixSecond);

        const cv::Mat projectionMatFirst = cameraMatrix * cv::Mat::eye(3,4, CV_64FC1);
        const cv::Mat projectionMatSecond = cameraMatrix * transformMatrixSecond;

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
        const Eigen::Affine3f estimatedTransformInverse = estimatedTransform.inverse();

        //compute error and update transform
        const float errorFirst = computeReprojectionError(detIt->firstPixels, triangulatedPoints3D,
                                                          Eigen::Affine3f::Identity(), cameraMatrix);
        const float errorSecond = computeReprojectionError(detIt->secondPixels, triangulatedPoints3D,
                                                           estimatedTransform, cameraMatrix);

        const float errorSecondInv = computeReprojectionError(detIt->secondPixels, triangulatedPoints3D,
                                                              estimatedTransformInverse, cameraMatrix);

        const float error = (errorFirst + errorSecond)/2.0f;

        qDebug() << "error: " << detIt.key().first << errorFirst;
        qDebug() << "error: " << detIt.key().second << errorSecond;
        qDebug() << "error second inverse: " << detIt.key().second << errorSecondInv;

        if (error < detIt->reprojectionError)
        {
            detIt->reprojectionError = error;
            detIt->transform = estimatedTransform;
            detIt->triangulatedPoints = triangulatedPoints3D;

            m_cameraSettings[detIt.key().second]->setTranslation(cv::Vec3f(translation.at<double>(0), translation.at<double>(1), translation.at<double>(2)));
            m_cameraSettings[detIt.key().second]->setRotation(cv::Vec3f(rotation.at<double>(0), rotation.at<double>(1), rotation.at<double>(2)));

            std::cout << "first estimation:" << std::endl;
            std::cout << "ROTATION: "
                      << rotation.at<double>(0) * 180.0/M_PI << "° "
                      << rotation.at<double>(1) * 180.0/M_PI << "° "
                      << rotation.at<double>(2) * 180.0/M_PI << "°" << std::endl;
            std::cout << "TRANSLATION: "
                      << translation.at<double>(0) << " "
                      << translation.at<double>(1) << " "
                      << translation.at<double>(2) << std::endl;
        }
    }

    //check if we can go to another stage of calibration
    auto cameraIDs = m_cameraSettings.keys();

    QVector<QUuid> discovered;
    std::queue<QUuid> pointQueue;

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
        const QUuid currentNode = pointQueue.front();
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
        qDebug() << "transform for camera" << disc << "ready for next stage";
    }

    //bundle adjustment stage
    //TODO for more than 2 cams;
    auto &data = m_observedDetections.first();
    cv::Mat cameraMatrix;
    m_camData.cameraMatrix.convertTo(cameraMatrix, CV_64FC1);

    const cv::Mat projectionMatFirst = cameraMatrix * cv::Mat::eye(3,4, CV_64FC1);

    double* dataCamFirst = data.cameraFirstParams(m_camData.cameraMatrix.at<double>(0,0));
    double* dataCamSecond = data.cameraSecondParams(m_camData.cameraMatrix.at<double>(0,0));

    float leftToMiddleDist = cv::norm(m_wandPoints[1] - m_wandPoints[0]);
    float leftToRightDist = cv::norm(m_wandPoints[2] - m_wandPoints[0]);
    float middleToRightDist = cv::norm(m_wandPoints[2] - m_wandPoints[1]);

    ceres::Problem problem;


    for (int i = 0; i < data.firstPixels.size(); i = i + m_wandPoints.size()) {
        ceres::CostFunction* costLeftFirstFunction =
            PointDistanceError::Create(leftToMiddleDist, middleToRightDist, leftToRightDist);

        problem.AddResidualBlock(costLeftFirstFunction, new ceres::CauchyLoss(0.5),
                                 &data.triangulatedPoints[i].x,
                                 &data.triangulatedPoints[i+1].x,
                                 &data.triangulatedPoints[i+2].x);
    }


    for (int i = 0; i < data.firstPixels.size(); ++i) {
        ceres::CostFunction* costFirstFunction =
            ReprojectionError::Create(data.firstPixels[i], cameraMatrix);

        problem.AddResidualBlock(costFirstFunction, new ceres::CauchyLoss(0.5), dataCamFirst,
                                 &data.triangulatedPoints[i].x);

        ceres::CostFunction* costSecondFunction =
            ReprojectionError::Create(data.secondPixels[i], cameraMatrix);

        problem.AddResidualBlock(costSecondFunction, new ceres::CauchyLoss(0.5), dataCamSecond,
                                 &data.triangulatedPoints[i].x);
    }

    problem.SetParameterLowerBound(dataCamFirst, 0, dataCamFirst[0] - M_PI/2);
    problem.SetParameterUpperBound(dataCamFirst, 0, dataCamFirst[0] + M_PI/2);
    problem.SetParameterLowerBound(dataCamFirst, 1, dataCamFirst[1] - M_PI/2);
    problem.SetParameterUpperBound(dataCamFirst, 1, dataCamFirst[1] + M_PI/2);
    problem.SetParameterLowerBound(dataCamFirst, 2, dataCamFirst[2] - M_PI/2);
    problem.SetParameterUpperBound(dataCamFirst, 2, dataCamFirst[2] + M_PI/2);

    problem.SetParameterLowerBound(dataCamSecond, 0, dataCamSecond[0] - M_PI/2);
    problem.SetParameterUpperBound(dataCamSecond, 0, dataCamSecond[0] + M_PI/2);
    problem.SetParameterLowerBound(dataCamSecond, 1, dataCamSecond[1] - M_PI/2);
    problem.SetParameterUpperBound(dataCamSecond, 1, dataCamSecond[1] + M_PI/2);
    problem.SetParameterLowerBound(dataCamSecond, 2, dataCamSecond[2] - M_PI/2);
    problem.SetParameterUpperBound(dataCamSecond, 2, dataCamSecond[2] + M_PI/2);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = QThread::idealThreadCount();
    options.num_linear_solver_threads = QThread::idealThreadCount();
    options.max_num_iterations = 300;
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    //options.use_nonmonotonic_steps = true;

    /*
    for (size_t i = 0; i < data.triangulatedPoints.size(); ++i)
    {
        std::cout << i << " :" << data.triangulatedPoints[i] << std::endl;
    }
    */

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    std::cout << "ROTATION AFTER: "
              << data.cameraFirstData[0] * 180.0/M_PI << " "
              << data.cameraFirstData[1] * 180.0/M_PI << " "
              << data.cameraFirstData[2] * 180.0/M_PI << std::endl;
    std::cout << "TRANSLATION AFTER: "
              << data.cameraFirstData[3] << " "
              << data.cameraFirstData[4] << " " << data.cameraFirstData[5] << std::endl;

    std::cout << "ROTATION AFTER: "
              << data.cameraSecondData[0] * 180.0/M_PI << " "
              << data.cameraSecondData[1] * 180.0/M_PI << " "
              << data.cameraSecondData[2] * 180.0/M_PI << std::endl;
    std::cout << "TRANSLATION AFTER: "
              << data.cameraSecondData[3] << " "
              << data.cameraSecondData[4] << " "
              << data.cameraSecondData[5] << std::endl;

    /*
    for (size_t i = 0; i < data.triangulatedPoints.size(); ++i)
    {
        std::cout << i << " :" << data.triangulatedPoints[i] << std::endl;
    }
    */

    {
        cv::Affine3f tr(cv::Vec3f(data.cameraFirstData[0], data.cameraFirstData[1], data.cameraFirstData[2]),
                        cv::Vec3f(data.cameraFirstData[3], data.cameraFirstData[4], data.cameraFirstData[5]));

        cv::Affine3f inv = tr.inv();

        m_cameraSettings[m_observedDetections.firstKey().first]->setTranslation(inv.translation());
        m_cameraSettings[m_observedDetections.firstKey().first]->setRotation(inv.rvec());
    }

    {
        cv::Affine3f tr(cv::Vec3f(data.cameraSecondData[0], data.cameraSecondData[1], data.cameraSecondData[2]),
                        cv::Vec3f(data.cameraSecondData[3], data.cameraSecondData[4], data.cameraSecondData[5]));

        cv::Affine3f inv = tr.inv();

        m_cameraSettings[m_observedDetections.firstKey().second]->setTranslation(inv.translation());
        m_cameraSettings[m_observedDetections.firstKey().second]->setRotation(inv.rvec());
    }

    finished = true;
}

float WandCalibration::computeReprojectionError(const std::vector<cv::Point2f> &pixels, const std::vector<cv::Point3d> &triangulatedPoints,
                                                const Eigen::Affine3f &estTransform, cv::Mat cameraMatrix)
{
    //TODO remove
    std::vector<cv::Point3f> triangulatedPtsFloat;

    cv::Affine3f cvEstTransform(estTransform);

    for (auto &pt : triangulatedPoints)
    {
        triangulatedPtsFloat.push_back(cvEstTransform * cv::Point3f(pt.x, pt.y, pt.z));
    }

    std::vector<cv::Point2f> projPixels;
    cv::projectPoints(triangulatedPtsFloat, cv::Vec3f::zeros(), cv::Vec3f::zeros(),
                      cameraMatrix, cv::noArray(), projPixels);
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

}
