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

#include <eigen3/Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/affine.hpp>

#include <limits>
#include <iostream>

WandCalibration::WandCalibration(QMap<int, std::shared_ptr<CameraSettings> > &cameraSettings,
                                 RPIMoCap::CameraParams camData, QObject *parent)
    : QObject(parent)
    , m_cameraSettings(cameraSettings)
    , m_camData(camData)
{
    m_wandPoints.push_back({-25.0,0.0,0.0});
    m_wandPoints.push_back({10.0,0.0,0.0});
    m_wandPoints.push_back({25.0,0.0,0.0});
    m_wandPoints.push_back({10.0,5.0,0.0});
}

void WandCalibration::addFrame(const QMap<int, std::vector<cv::Point2f> > &points)
{
    std::vector<std::pair<int, std::vector<cv::Point2f>>> detectedPoints;

    //detect points (order)
    for (auto &camID : points.keys())
    {
        if (auto detPts = detect4pWand(points[camID]))
        {
            detectedPoints.push_back({camID, detPts.value()});
        }
    }

    //add detected points to observation
    for (size_t i = 0; i < detectedPoints.size(); ++i)
    {
        for (size_t j = i + 1; j < detectedPoints.size(); ++j)
        {
            //TODO add only if points are diferent enought from previous detections
            //we don't want many detections from same position
            ObsDetection &obsDet = m_observedDetections[{detectedPoints[i].first,detectedPoints[j].first}];
            obsDet.firstPixels.insert(obsDet.firstPixels.end(), detectedPoints[i].second.begin(), detectedPoints[i].second.end());
            obsDet.secondPixels.insert(obsDet.secondPixels.end(), detectedPoints[j].second.begin(), detectedPoints[j].second.end());
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

        std::vector<cv::Point3f> triangulatedPoints3D;

        for (size_t i = 0; i < triangulatedPoints.cols; ++i)
        {
            const float w = triangulatedPoints.at<float>(3,i);
            triangulatedPoints3D.push_back({triangulatedPoints.at<float>(0,i)/w,
                                            triangulatedPoints.at<float>(1,i)/w,
                                            triangulatedPoints.at<float>(2,i)/w});
        }

        const float scale = computeScale(transformMatrixSecond, triangulatedPoints3D);


        Eigen::Affine3f estimatedTransform(cv::Affine3d(rotation, translation).cast<float>());

        const float errorFirst = computeReprojectionError(detIt->firstPixels, triangulatedPoints3D, Eigen::Affine3f::Identity());
        const float errorSecond = computeReprojectionError(detIt->secondPixels, triangulatedPoints3D, estimatedTransform);

        const float error = (errorFirst + errorSecond)/2.0f;

        std::cout << "error: " << detIt.key().first << errorFirst << std::endl;
        std::cout << "error: " << detIt.key().first << errorSecond << std::endl;

        estimatedTransform.translation() *= scale;

        if (error < detIt->reprojectionError)
        {
            detIt->reprojectionError = error;
            detIt->transform = estimatedTransform;

            m_cameraSettings[detIt.key().second]->setTransform(estimatedTransform);
        }
    }
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

float WandCalibration::computeReprojectionError(const std::vector<cv::Point2f> &pixels, const std::vector<cv::Point3f> &triangulatedPoints,
                                                const Eigen::Affine3f &estTransform)
{
    cv::Affine3f cvEstTransform(estTransform);

    cv::Mat rVec;
    cv::Rodrigues(cvEstTransform.rotation(), rVec);

    cv::Vec3f tVec(estTransform.translation().x(),estTransform.translation().y(),estTransform.translation().z());

    std::vector<cv::Point2f> projPixels;
    cv::projectPoints(triangulatedPoints, rVec, tVec, m_camData.cameraMatrix, m_camData.distortionCoeffs, projPixels);
    assert(projPixels.size() == pixels.size());

    float sum = 0.0f;
    for (size_t i = 0; i < pixels.size(); ++i)
    {
        sum += cv::norm(projPixels[i] - pixels[i]);
    }

    return sum/pixels.size();
}

bool WandCalibration::haveAllExtrinsicGuess()
{
    for (const auto &guess : m_extrinsicGuess)
    {
        if (guess.state == CalibrationState::Unknown)
        {
            return false;
        }
    }
    return true;
}

float WandCalibration::computeScale(const cv::Mat &transformEstimation, std::vector<cv::Point3f> &triangulatedPoints)
{
    assert(triangulatedPoints.size() % m_wandPoints.size() == 0);

    float scale = 0.0f;
    const int numOfObservations = triangulatedPoints.size()/m_wandPoints.size();
    for (size_t i = 1; i < m_wandPoints.size(); ++i)
    {
        const float realDistance = cv::norm(m_wandPoints[i] - m_wandPoints[i -1]);

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
