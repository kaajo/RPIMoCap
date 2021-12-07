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

#include "Server/wandcalibration.h"

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

WandCalibration::WandCalibration(QObject *parent)
    : QObject(parent)
{

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

        cv::Mat relativeTransform = computeBasicRelativeTransform(detIt->first.pixels, detIt->second.pixels, cameraMatrix);

        const cv::Mat projectionMatFirst = cameraMatrix * cv::Mat::eye(3,4, CV_64FC1);
        const cv::Mat projectionMatSecond = cameraMatrix * relativeTransform;

        cv::Mat triangulatedPoints;
        cv::triangulatePoints(projectionMatFirst, projectionMatSecond, detIt->first.pixels, detIt->second.pixels, triangulatedPoints);

        std::vector<cv::Point3d> triangulatedPoints3D;

        for (int i = 0; i < triangulatedPoints.cols; ++i)
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

        detIt->first.rVec = Eigen::Vector3f::Zero();
        detIt->first.tVec = Eigen::Vector3f::Zero();
        detIt->second.rVec = Eigen::Vector3f(transformScaled.rvec()[0],
                                            transformScaled.rvec()[1],
                                            transformScaled.rvec()[2]);
        detIt->second.tVec = Eigen::Vector3f(transformScaled.translation()[0],
                                            transformScaled.translation()[1],
                                            transformScaled.translation()[2]);

        qDebug() << "error: " << detIt.key().first << detIt->first.reprojectionError(detIt->triangulatedPoints, detIt->cameraMatrix);
        std::cout << "rvec: " << detIt->first.rVec * 180.0/M_PI << std::endl;
        std::cout << "tvec: " << detIt->first.tVec << std::endl;
        qDebug() << "error: " << detIt.key().second << detIt->second.reprojectionError(detIt->triangulatedPoints, detIt->cameraMatrix);
        std::cout << "rvec: " << detIt->second.rVec * 180.0/M_PI << std::endl;
        std::cout << "tvec: " << detIt->second.tVec << std::endl;
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

        detIt->first.rVec = {fRVec[0], fRVec[1], fRVec[2]};
        detIt->second.rVec = {sRVec[0], sRVec[1], sRVec[2]};

        detIt->first.tVec = fTransform.translation();
        detIt->second.tVec = sTransform.translation();

        // Triangulate points
        cv::Mat triangulatedPoints;

        auto fp = detIt->first.projectionMatrix(detIt->cameraMatrix);
        auto sp = detIt->second.projectionMatrix(detIt->cameraMatrix);

        cv::triangulatePoints(fp, sp, detIt->first.pixels, detIt->second.pixels, triangulatedPoints);

        std::vector<cv::Point3d> triangulatedPoints3D;

        for (int i = 0; i < triangulatedPoints.cols; ++i)
        {
            const float w = triangulatedPoints.at<float>(3,i);
            triangulatedPoints3D.push_back({triangulatedPoints.at<float>(0,i)/w,
                                            triangulatedPoints.at<float>(1,i)/w,
                                            triangulatedPoints.at<float>(2,i)/w});
        }

        detIt->triangulatedPoints = triangulatedPoints3D;

        qDebug() << "error: " << detIt.key().first << detIt->first.reprojectionError(detIt->triangulatedPoints, detIt->cameraMatrix);
        std::cout << "rvec: " << detIt->first.rVec * 180.0/M_PI << std::endl;
        std::cout << "tvec: " << detIt->first.tVec << std::endl;
        qDebug() << "error: " << detIt.key().second << detIt->second.reprojectionError(detIt->triangulatedPoints, detIt->cameraMatrix);
        std::cout << "rvec: " << detIt->second.rVec * 180.0/M_PI << std::endl;
        std::cout << "tvec: " << detIt->second.tVec << std::endl;
    }

    //init distances between wand points
    float leftToMiddleDist = cv::norm(m_wandPoints[1] - m_wandPoints[0]);
    float leftToRightDist = cv::norm(m_wandPoints[2] - m_wandPoints[0]);
    float middleToRightDist = cv::norm(m_wandPoints[2] - m_wandPoints[1]);

    //get data from every camera

    QMap<QUuid, std::array<double, 6>> cameraData;

    for (auto detIt = m_observedDetections.begin(); detIt != m_observedDetections.end(); ++detIt)
    {
        if (!cameraData.contains(detIt.key().first))
        {
            cameraData.insert(detIt.key().first, detIt->first.ceresParams());
        }

        if (!cameraData.contains(detIt.key().second))
        {
            cameraData.insert(detIt.key().second, detIt->second.ceresParams());
        }
    }

    //add observations
    for (auto detIt = m_observedDetections.begin(); detIt != m_observedDetections.end(); ++detIt)
    {
        double* dataCamFirst = &cameraData[detIt.key().first][0];
        double* dataCamSecond = &cameraData[detIt.key().second][0];

        for (size_t i = 0; i < detIt->first.pixels.size(); i = i + m_wandPoints.size()) {
            ceres::CostFunction* costLeftFirstFunction =
                PointDistanceError::Create(leftToMiddleDist, middleToRightDist, leftToRightDist);

            problem.AddResidualBlock(costLeftFirstFunction, new ceres::CauchyLoss(0.5),
                                     &detIt->triangulatedPoints[i].x,
                                     &detIt->triangulatedPoints[i+1].x,
                                     &detIt->triangulatedPoints[i+2].x);
        }

        for (size_t i = 0; i < detIt->first.pixels.size(); ++i) {
            ceres::CostFunction* costFirstFunction =
                ReprojectionError::Create(detIt->first.pixels[i], detIt->cameraMatrix);

            problem.AddResidualBlock(costFirstFunction, new ceres::CauchyLoss(0.5), dataCamFirst,
                                     &detIt->triangulatedPoints[i].x);

            ceres::CostFunction* costSecondFunction =
                ReprojectionError::Create(detIt->second.pixels[i], detIt->cameraMatrix);

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
            double* dataCamFirst = &cameraData[detIt.key().first][0];
            cv::Affine3f tr(cv::Vec3f(dataCamFirst[0], dataCamFirst[1], dataCamFirst[2]),
                            cv::Vec3f(dataCamFirst[3], dataCamFirst[4], dataCamFirst[5]));
            cv::Affine3f inv = tr.inv();

            detIt->first.rVec = Eigen::Vector3f(inv.rvec()[0], inv.rvec()[1], inv.rvec()[2]);
            detIt->first.tVec = Eigen::Vector3f(inv.translation()[0], inv.translation()[1], inv.translation()[2]);

            m_cameraSettings[detIt.key().first]->setRotation(Eigen::Vector3d(inv.rvec()[0], inv.rvec()[1], inv.rvec()[2]));
            m_cameraSettings[detIt.key().first]->setTranslation(Eigen::Vector3d(inv.translation()[0], inv.translation()[1], inv.translation()[2]));
        }

        {
            double* dataCamSecond = &cameraData[detIt.key().second][0];
            cv::Affine3f tr(cv::Vec3f(dataCamSecond[0], dataCamSecond[1], dataCamSecond[2]),
                            cv::Vec3f(dataCamSecond[3], dataCamSecond[4], dataCamSecond[5]));
            cv::Affine3f inv = tr.inv();

            detIt->second.rVec = Eigen::Vector3f(inv.rvec()[0], inv.rvec()[1], inv.rvec()[2]);
            detIt->second.tVec = Eigen::Vector3f(inv.translation()[0], inv.translation()[1], inv.translation()[2]);

            m_cameraSettings[detIt.key().second]->setRotation(Eigen::Vector3d(inv.rvec()[0], inv.rvec()[1], inv.rvec()[2]));
            m_cameraSettings[detIt.key().second]->setTranslation(Eigen::Vector3d(inv.translation()[0], inv.translation()[1], inv.translation()[2]));
        }

        qDebug() << "reprojection error after second stage";
        qDebug() << "error: " << detIt.key().first << detIt->first.reprojectionError(detIt->triangulatedPoints, detIt->cameraMatrix);
        std::cout << "rvec: " << detIt->first.rVec * 180.0/M_PI << std::endl;
        std::cout << "tvec: " << detIt->first.tVec << std::endl;
        qDebug() << "error: " << detIt.key().second << detIt->second.reprojectionError(detIt->triangulatedPoints, detIt->cameraMatrix);
        std::cout << "rvec: " << detIt->second.rVec * 180.0/M_PI << std::endl;
        std::cout << "tvec: " << detIt->second.tVec << std::endl;
    }

    finished = true;
}

QMap<QUuid, Eigen::Affine3f> WandCalibration::relativeToGlobalTransforms(const QMap<std::pair<QUuid, QUuid>, ObservationPair> &detections)
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

void WandCalibration::startCalib(bool start, WandCalibration::Settings settings, InputData data)
{
    started = start;
    finished = !start;

    if (!start) {
        qDebug() << "end calibration";
        return;
    }

    switch (settings.calibType) {
    case WandCalibration::Type::Full:
        m_camData = data.camParams;
        m_cameraSettings = data.cameraSettings;
        m_wandPoints = data.wandPoints;
        m_observedDetections.clear();
        m_minCalibObservations = settings.framesPerCamera;
        break;
    case WandCalibration::Type::Refine:
        m_minCalibObservations += settings.framesPerCamera;
        break;
    }
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

bool WandCalibration::isCameraPairReady(const ObservationPair &detections)
{
    const int detectedFrames = std::min(detections.first.pixels.size(),
                                        detections.second.pixels.size());

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
            auto &obsDet = m_observedDetections[{wandPoints[i].first,wandPoints[j].first}];

            if (isCameraPairReady(obsDet))
            {
                continue;
            }

            float maxDiff = 0.0;

            if (!obsDet.first.pixels.empty())
            {
                for (size_t wandIndex = 0; wandIndex < m_wandPoints.size(); ++wandIndex)
                {
                    const float diff = cv::norm(wandPoints[i].second[wandIndex] - obsDet.first.pixels[obsDet.first.pixels.size() - m_wandPoints.size() + wandIndex]);
                    const float diff2 = cv::norm(wandPoints[j].second[wandIndex] - obsDet.second.pixels[obsDet.second.pixels.size() - m_wandPoints.size() + wandIndex]);
                    if (std::max(diff, diff2) > maxDiff)
                    {
                        maxDiff = std::max(diff, diff2);
                    }
                }
            }

            if (maxDiff > 5.0 || obsDet.first.pixels.empty() || obsDet.second.pixels.empty())
            {
                obsDet.first.pixels.insert(obsDet.first.pixels.end(), wandPoints[i].second.begin(), wandPoints[i].second.end());
                obsDet.second.pixels.insert(obsDet.second.pixels.end(), wandPoints[j].second.begin(), wandPoints[j].second.end());

                qDebug() << "add observation:" << obsDet.first.pixels.size()/m_wandPoints.size() << "/" << m_minCalibObservations;
            }
        }
    }
}

cv::Mat WandCalibration::computeBasicRelativeTransform(std::vector<cv::Point2f> firstpixels, std::vector<cv::Point2f> secondpixels, cv::Mat &cameraMatrix)
{
    const cv::Mat essential = cv::findEssentialMat(firstpixels, secondpixels, cameraMatrix, cv::RANSAC, 0.999, 0.1);

    cv::Mat rotMat, tVec;
    cv::recoverPose(essential, firstpixels, secondpixels, cameraMatrix, rotMat, tVec);

    cv::Mat relativeTransform;
    cv::hconcat(rotMat, tVec, relativeTransform);
    return relativeTransform;
}

}
