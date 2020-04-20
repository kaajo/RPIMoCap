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

void WandCalibration::addFrame(const std::vector<std::pair<QUuid, std::vector<cv::Point2f>>> &points)
{
    if (finished) return;

    std::vector<std::pair<QUuid, std::vector<cv::Point2f>>> detectedPoints;

    //detect points (reorder)
    for (auto &detection : points)
    {
        if (auto detPts = WandDetector::detect3pWand(detection.second))
        {
            detectedPoints.push_back({detection.first, detPts.value()});
        }
    }

    addObservations(detectedPoints);

    for (auto detIt = m_observedDetections.begin(); detIt != m_observedDetections.end(); ++detIt)
    {
        if (!isCameraPairReady(*detIt) || detIt->reprojectionError() < m_errorTreshold)
        {
            continue;
        }

        cv::Mat cameraMatrix;
        m_camData.cameraMatrix.convertTo(cameraMatrix, CV_64FC1);
        detIt->cameraMatrix = cameraMatrix;

        cv::Mat relativeTransform = computeBasicRelativeTransform(detIt->firstPixels, detIt->secondPixels, cameraMatrix);

        const cv::Mat projectionMatFirst = cameraMatrix * cv::Mat::eye(3,4, CV_64FC1);
        const cv::Mat projectionMatSecond = cameraMatrix * relativeTransform;

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

        const float scale = computeScale(triangulatedPoints3D);

        //scale scene
        for(auto &triangulatedPoint : triangulatedPoints3D)
        {
            triangulatedPoint *= scale;
        }

        detIt->triangulatedPoints = triangulatedPoints3D;

        cv::Affine3d transform(relativeTransform);
        cv::Affine3f transformScaled(transform.rotation(),
                                     transform.translation() * scale);

        detIt->firstRVec = Eigen::Vector3f::Zero();
        detIt->firstTVec = Eigen::Vector3f::Zero();
        detIt->secondRVec = Eigen::Vector3f(transformScaled.rvec()[0],
                                            transformScaled.rvec()[1],
                                            transformScaled.rvec()[2]);
        detIt->secondTVec = Eigen::Vector3f(transformScaled.translation()[0],
                                            transformScaled.translation()[1],
                                            transformScaled.translation()[2]);

        qDebug() << "error: " << detIt.key().first << detIt->firstReprojectionError();
        std::cout << "rvec: " << detIt->firstRVec * 180.0/M_PI << std::endl;
        std::cout << "tvec: " << detIt->firstTVec << std::endl;
        qDebug() << "error: " << detIt.key().second << detIt->secondReprojectionError();
        std::cout << "rvec: " << detIt->secondRVec * 180.0/M_PI << std::endl;
        std::cout << "tvec: " << detIt->secondTVec << std::endl;
    }


    if (isReadyForPreciseStage())
    {
        qDebug() << "calibration ready for bundle adjustment";
    }
    else
    {
        return;
    }

    //bundle adjustment stage
    ceres::Problem problem;

    //remove "not ready" detections
    for (auto it = m_observedDetections.begin(); it != m_observedDetections.end();)
    {
        if (!isCameraPairReady(*it))
        {
            it = m_observedDetections.erase(it);
        }
        else
        {
            ++it;
        }
    }

    //convert relative transform to absolute transform
    auto absoluteTransforms = relativeToGlobalTransforms(m_observedDetections);

    qDebug() << "global transforms:";

    for (auto &t : absoluteTransforms)
    {
        std::cout << "rvec: " << cv::Affine3f(t).rvec()  * 180.0/M_PI << std::endl;
        std::cout << "tvec: " << t.translation() << std::endl;
    }

    qDebug() << "global transforms after:";

    for (auto detIt = m_observedDetections.begin(); detIt != m_observedDetections.end(); ++detIt)
    {
        auto fTransform = absoluteTransforms[detIt.key().first];
        auto sTransform = absoluteTransforms[detIt.key().second];

        auto fRVec = cv::Affine3f(fTransform).rvec();
        auto sRVec = cv::Affine3f(sTransform).rvec();

        detIt->firstRVec = {fRVec[0], fRVec[1], fRVec[2]};
        detIt->secondRVec = {sRVec[0], sRVec[1], sRVec[2]};

        detIt->firstTVec = fTransform.translation();
        detIt->secondTVec = sTransform.translation();

        // Triangulate points
        cv::Mat triangulatedPoints;

        auto fp = detIt->firstProjMat();
        auto sp = detIt->secondProjMat();

        cv::triangulatePoints(fp, sp, detIt->firstPixels, detIt->secondPixels, triangulatedPoints);

        std::vector<cv::Point3d> triangulatedPoints3D;

        for (size_t i = 0; i < triangulatedPoints.cols; ++i)
        {
            const float w = triangulatedPoints.at<float>(3,i);
            triangulatedPoints3D.push_back({triangulatedPoints.at<float>(0,i)/w,
                                            triangulatedPoints.at<float>(1,i)/w,
                                            triangulatedPoints.at<float>(2,i)/w});
        }

        detIt->triangulatedPoints = triangulatedPoints3D;

        qDebug() << "error: " << detIt.key().first << detIt->firstReprojectionError();
        std::cout << "rvec: " << detIt->firstRVec * 180.0/M_PI << std::endl;
        std::cout << "tvec: " << detIt->firstTVec << std::endl;
        qDebug() << "error: " << detIt.key().second << detIt->secondReprojectionError();
        std::cout << "rvec: " << detIt->secondRVec * 180.0/M_PI << std::endl;
        std::cout << "tvec: " << detIt->secondTVec << std::endl;
    }

    //init distances between wand points
    float leftToMiddleDist = cv::norm(m_wandPoints[1] - m_wandPoints[0]);
    float leftToRightDist = cv::norm(m_wandPoints[2] - m_wandPoints[0]);
    float middleToRightDist = cv::norm(m_wandPoints[2] - m_wandPoints[1]);

    //get data from every camera

    QMap<QUuid, double*> cameraData;

    for (auto detIt = m_observedDetections.begin(); detIt != m_observedDetections.end(); ++detIt)
    {
        if (!cameraData.contains(detIt.key().first))
        {
            cameraData.insert(detIt.key().first, detIt->cameraFirstParams(m_camData.cameraMatrix.at<double>(0,0)));
        }

        if (!cameraData.contains(detIt.key().second))
        {
            cameraData.insert(detIt.key().second, detIt->cameraSecondParams(m_camData.cameraMatrix.at<double>(0,0)));
        }
    }

    //add observations
    for (auto detIt = m_observedDetections.begin(); detIt != m_observedDetections.end(); ++detIt)
    {
        double* dataCamFirst = cameraData[detIt.key().first];
        double* dataCamSecond = cameraData[detIt.key().second];

        for (int i = 0; i < detIt->firstPixels.size(); i = i + m_wandPoints.size()) {
            ceres::CostFunction* costLeftFirstFunction =
                PointDistanceError::Create(leftToMiddleDist, middleToRightDist, leftToRightDist);

            problem.AddResidualBlock(costLeftFirstFunction, new ceres::CauchyLoss(0.5),
                                     &detIt->triangulatedPoints[i].x,
                                     &detIt->triangulatedPoints[i+1].x,
                                     &detIt->triangulatedPoints[i+2].x);
        }

        for (int i = 0; i < detIt->firstPixels.size(); ++i) {
            ceres::CostFunction* costFirstFunction =
                ReprojectionError::Create(detIt->firstPixels[i], detIt->cameraMatrix);

            problem.AddResidualBlock(costFirstFunction, new ceres::CauchyLoss(0.5), dataCamFirst,
                                     &detIt->triangulatedPoints[i].x);

            ceres::CostFunction* costSecondFunction =
                ReprojectionError::Create(detIt->secondPixels[i], detIt->cameraMatrix);

            problem.AddResidualBlock(costSecondFunction, new ceres::CauchyLoss(0.5), dataCamSecond,
                                     &detIt->triangulatedPoints[i].x);
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
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = QThread::idealThreadCount();
    options.num_linear_solver_threads = QThread::idealThreadCount();
    options.max_num_iterations = 300;
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    //options.use_nonmonotonic_steps = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    for (auto detIt = m_observedDetections.begin(); detIt != m_observedDetections.end(); ++detIt)
    {
        {
            double* dataCamFirst = cameraData[detIt.key().first];
            cv::Affine3f tr(cv::Vec3f(dataCamFirst[0], dataCamFirst[1], dataCamFirst[2]),
                            cv::Vec3f(dataCamFirst[3], dataCamFirst[4], dataCamFirst[5]));
            cv::Affine3f inv = tr.inv();

            detIt->firstRVec = Eigen::Vector3f(inv.rvec()[0], inv.rvec()[1], inv.rvec()[2]);
            detIt->firstTVec = Eigen::Vector3f(inv.translation()[0], inv.translation()[1], inv.translation()[2]);

            m_cameraSettings[detIt.key().first]->setRotation(inv.rvec());
            m_cameraSettings[detIt.key().first]->setTranslation(inv.translation());
        }

        {
            double* dataCamSecond = cameraData[detIt.key().second];
            cv::Affine3f tr(cv::Vec3f(dataCamSecond[0], dataCamSecond[1], dataCamSecond[2]),
                            cv::Vec3f(dataCamSecond[3], dataCamSecond[4], dataCamSecond[5]));
            cv::Affine3f inv = tr.inv();

            detIt->secondRVec = Eigen::Vector3f(inv.rvec()[0], inv.rvec()[1], inv.rvec()[2]);
            detIt->secondTVec = Eigen::Vector3f(inv.translation()[0], inv.translation()[1], inv.translation()[2]);

            m_cameraSettings[detIt.key().second]->setRotation(inv.rvec());
            m_cameraSettings[detIt.key().second]->setTranslation(inv.translation());
        }

        qDebug() << "reprojection error after second stage";
        qDebug() << "error: " << detIt.key().first << detIt->firstReprojectionError();
        std::cout << "rvec: " << detIt->firstRVec * 180.0/M_PI << std::endl;
        std::cout << "tvec: " << detIt->firstTVec << std::endl;
        qDebug() << "error: " << detIt.key().second << detIt->secondReprojectionError();
        std::cout << "rvec: " << detIt->secondRVec * 180.0/M_PI << std::endl;
        std::cout << "tvec: " << detIt->secondTVec << std::endl;
    }

    finished = true;
}

QMap<QUuid, Eigen::Affine3f> WandCalibration::relativeToGlobalTransforms(const QMap<std::pair<QUuid, QUuid>, ObsDetection> &detections)
{
    QVector<QUuid> discovered;
    std::queue<QUuid> pointQueue;
    QMap<QUuid, Eigen::Affine3f> transforms;

    discovered.push_back(detections.firstKey().first);
    pointQueue.push(detections.firstKey().first);
    transforms[detections.firstKey().first] = Eigen::Affine3f::Identity();

    while (!pointQueue.empty())
    {
        const QUuid currentNode = pointQueue.front();
        pointQueue.pop();

        //search for all nodes with this ID
        for (auto detIt = detections.begin(); detIt != detections.end(); ++detIt)
        {
            if (detIt.key().first == currentNode && !discovered.contains(detIt.key().second))
            {
                pointQueue.push(detIt.key().second);
                discovered.push_back(detIt.key().second);

                auto tf = detIt->relativeTransform() * transforms[detIt.key().first];
                transforms.insert(detIt.key().second, tf);
            }

            if (detIt.key().second == currentNode && !discovered.contains(detIt.key().first))
            {
                pointQueue.push(detIt.key().first);
                discovered.push_back(detIt.key().first);

                auto tf = detIt->relativeTransform().inverse() * transforms[detIt.key().second];
                transforms.insert(detIt.key().first, tf); //inverse (second -> first)
            }
        }
    }

    return transforms;
}

float WandCalibration::computeScale(std::vector<cv::Point3d> &triangulatedPoints)
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

bool WandCalibration::isCameraPairReady(const ObsDetection &detections)
{
    const int detectedFrames = std::min(detections.firstPixels.size(),
                                        detections.secondPixels.size());

    return detectedFrames > m_minCalibObservations * m_wandPoints.size();
}

bool WandCalibration::isReadyForPreciseStage()
{
    //check if we can go to another stage of calibration
    auto cameraIDs = m_cameraSettings.keys();

    QVector<QUuid> discovered;
    std::queue<QUuid> pointQueue;

    //find first good node and mark it as reference camera
    for (auto detIt = m_observedDetections.begin(); detIt != m_observedDetections.end(); ++detIt)
    {
        if (detIt->reprojectionError() < m_errorTreshold)
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
            if (detIt.key().first == currentNode && detIt->reprojectionError() < m_errorTreshold)
            {
                if (!discovered.contains(detIt.key().second))
                {
                    pointQueue.push(detIt.key().second);
                    discovered.push_back(detIt.key().second);
                }
            }

            if (detIt.key().second == currentNode && detIt->reprojectionError() < m_errorTreshold)
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
        return false;
    }

    return true;
}

void WandCalibration::addObservations(const std::vector<std::pair<QUuid, std::vector<cv::Point2f>>> &wandPoints)
{
    for (size_t i = 0; i < wandPoints.size(); ++i)
    {
        for (size_t j = i + 1; j < wandPoints.size(); ++j)
        {
            ObsDetection &obsDet = m_observedDetections[{wandPoints[i].first,wandPoints[j].first}];

            if (isCameraPairReady(obsDet))
            {
                continue;
            }

            float maxDiff = 0.0;

            if (!obsDet.firstPixels.empty())
            {
                for (int wandIndex = 0; wandIndex < m_wandPoints.size(); ++wandIndex)
                {
                    const float diff = cv::norm(wandPoints[i].second[wandIndex] - obsDet.firstPixels[obsDet.firstPixels.size() - m_wandPoints.size() + wandIndex]);
                    const float diff2 = cv::norm(wandPoints[j].second[wandIndex] - obsDet.secondPixels[obsDet.secondPixels.size() - m_wandPoints.size() + wandIndex]);
                    if (std::max(diff, diff2) > maxDiff)
                    {
                        maxDiff = std::max(diff, diff2);
                    }
                }
            }

            if (maxDiff > 5.0 || obsDet.firstPixels.empty() || obsDet.secondPixels.empty())
            {
                obsDet.firstPixels.insert(obsDet.firstPixels.end(), wandPoints[i].second.begin(), wandPoints[i].second.end());
                obsDet.secondPixels.insert(obsDet.secondPixels.end(), wandPoints[j].second.begin(), wandPoints[j].second.end());

                qDebug() << "add observation:" << obsDet.firstPixels.size()/m_wandPoints.size() << "/" << m_minCalibObservations;
            }
        }
    }
}

cv::Mat WandCalibration::computeBasicRelativeTransform(std::vector<cv::Point2f> firstPixels, std::vector<cv::Point2f> secondPixels, cv::Mat &cameraMatrix)
{
    const cv::Mat essential = cv::findEssentialMat(firstPixels, secondPixels, cameraMatrix, cv::RANSAC, 0.999, 0.1);

    cv::Mat rotMat, tVec;
    cv::recoverPose(essential, firstPixels, secondPixels, cameraMatrix, rotMat, tVec);

    cv::Mat relativeTransform;
    cv::hconcat(rotMat, tVec, relativeTransform);
    return relativeTransform;
}

}
