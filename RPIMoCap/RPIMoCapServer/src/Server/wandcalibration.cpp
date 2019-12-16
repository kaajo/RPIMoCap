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

    for (const auto &camSettings : m_cameraSettings)
    {
        m_extrinsicGuess[camSettings->id()].transform = std::nullopt;
    }

    m_wandPoints.push_back({-25.0,0.0,0.0});
    m_wandPoints.push_back({10.0,0.0,0.0});
    m_wandPoints.push_back({25.0,0.0,0.0});
    m_wandPoints.push_back({10.0,5.0,0.0});
}

void WandCalibration::addFrame(const QMap<int, std::vector<cv::Point2f> > &points)
{
    if (done) return;

    QVector<std::pair<int, std::vector<cv::Point2f>>> detectedPoints;

    for (auto &camID : points.keys())
    {
        if (auto detPts = detect4pWand(points[camID]))
        {
            detectedPoints.push_back({camID, detPts.value()});
        }
    }

    for (int i = 0; i < detectedPoints.size(); ++i)
    {
        for (int j = i + 1; j < detectedPoints.size(); ++j)
        {
            ObsDetection &obsDet = m_observedDetections[{detectedPoints[i].first,detectedPoints[j].first}];
            obsDet.firstPixels.insert(obsDet.firstPixels.end(), detectedPoints[i].second.begin(), detectedPoints[i].second.end());
            obsDet.secondPixels.insert(obsDet.secondPixels.end(), detectedPoints[j].second.begin(), detectedPoints[j].second.end());
        }
    }

    //find minimal point count detected
    int min = std::numeric_limits<int>::max();
    QMap<std::pair<int,int>,ObsDetection>::iterator detIt;
    for (detIt = m_observedDetections.begin(); detIt != m_observedDetections.end(); ++detIt)
    {
        std::cout << "ID: " << detIt.key().first << " pts: " << detIt->firstPixels.size() << std::endl;
        std::cout << "ID: " << detIt.key().second << " pts: " << detIt->secondPixels.size() << std::endl;

        if (detIt->firstPixels.size() < min)
        {
            min = detIt->firstPixels.size();
        }

        if (detIt->secondPixels.size() < min)
        {
            min = detIt->secondPixels.size();
        }
    }

    if (min > 200)
    {
        QMap<std::pair<int,int>,ObsDetection>::iterator detIt;
        for (detIt = m_observedDetections.begin(); detIt != m_observedDetections.end(); ++detIt)
        {
            cv::Mat rotVec, rotMat, translation;

            cv::Mat essential = cv::findEssentialMat(detIt->firstPixels, detIt->secondPixels, m_camData.cameraMatrix, cv::RANSAC, 0.999, 0.1);
            cv::recoverPose(essential, detIt->firstPixels, detIt->secondPixels, m_camData.cameraMatrix, rotMat, translation);
            cv::Rodrigues(rotMat, rotVec);

            std::cout << "rotation: " << rotVec.at<double>(0) * 180.0/M_PI
                      << " " << rotVec.at<double>(1) * 180.0/M_PI
                      << " " << rotVec.at<double>(2) * 180.0/M_PI << std::endl;

            std::cout << "translation: " << translation.at<double>(0)
                      << " " << translation.at<double>(1)
                      << " " << translation.at<double>(2) << std::endl;

            //compute scale
            cv::Mat floatCamMat;
            m_camData.cameraMatrix.convertTo(floatCamMat, CV_32FC1);
            cv::Mat projectionMatFirst = floatCamMat * cv::Mat::eye(3,4, CV_32FC1);

            cv::Mat tmp;
            cv::hconcat(rotMat, translation, tmp);
            cv::Mat projectionMatSecond = m_camData.cameraMatrix * tmp;
            projectionMatSecond.convertTo(projectionMatSecond, CV_32FC1);

            cv::Mat triangulatedPoints;
            cv::triangulatePoints(projectionMatFirst, projectionMatSecond, detIt->firstPixels, detIt->secondPixels, triangulatedPoints);

            std::vector<cv::Vec3f> triangulatedPoints3D;

            for (size_t i = 0; i < triangulatedPoints.cols; ++i)
            {
                const float w = triangulatedPoints.at<float>(3,i);
                triangulatedPoints3D.push_back({triangulatedPoints.at<float>(0,i)/w,
                                                triangulatedPoints.at<float>(1,i)/w,
                                                triangulatedPoints.at<float>(2,i)/w});
            }

            assert(triangulatedPoints3D.size() % m_wandPoints.size() == 0);

            float scale = 0.0;
            for (int i = 0; i < triangulatedPoints3D.size(); i = i+4)
            {
                scale += cv::norm(triangulatedPoints3D[i] - triangulatedPoints3D[i+1]);
            }

            scale = 35.0/(scale / (static_cast<double>(triangulatedPoints3D.size()/4.0)));

            translation *= scale;

            std::cout << "translation: " << translation.at<double>(0)
                      << " " << translation.at<double>(1)
                      << " " << translation.at<double>(2) << std::endl;

            m_cameraSettings[detIt.key().second]->setTransform(Eigen::Affine3f(cv::Affine3d(rotMat, translation).cast<float>()));
        }
    }


    /*
    auto frameGuess = frameExtGuess(points);

    if (frameGuess.empty())
    {
        return;
    }

    std::sort(frameGuess.begin(), frameGuess.end(), [](auto fg1, auto fg2){return fg1.cameraID < fg2.cameraID;});

    if (m_referenceCameraID == -1)
    {
        m_referenceCameraID = frameGuess.front().cameraID;
        m_extrinsicGuess[m_referenceCameraID] = ExtrGuessRes{m_referenceCameraID, 0.0f, Eigen::Affine3f::Identity()};
    }


    if (haveAllExtrinsicGuess())
    {
        const auto keys = m_extrinsicGuess.keys();

        std::cout << "not normalised extrinsic guess:" << std::endl;
        for (auto &guessKey : keys)
        {
            std::cout << m_extrinsicGuess[guessKey].transform.value().matrix() << std::endl;
        }

        //change transform relative to first one

        m_cameraSettings[keys[0]]->setTransform(Eigen::Affine3f::Identity());

        const Eigen::Affine3f firstTf = m_extrinsicGuess[keys[0]].transform.value();

        for (size_t i = 1; i < keys.size(); ++i)
        {
            m_cameraSettings[keys[i]]->setTransform(firstTf * m_extrinsicGuess[keys[i]].transform.value().inverse());
        }
    }
    */
}

std::optional<Eigen::Affine3f> WandCalibration::relativeTransform(const size_t camID, const std::vector<cv::Point2f> &pixels)
{
    if (pixels.size() != m_wandPoints.size())
    {
        return std::nullopt;
    }

    const std::vector<cv::Point2f> detPixels = detect4pWand(pixels).value();

    cv::Mat rVec;
    cv::Mat tVec;

    //    const bool solved = cv::solvePnPRansac(m_wandPoints,detPixels, m_camData.cameraMatrix,
    //                                           m_camData.distortionCoeffs, rVec, tVec, false, 300, 8.0, 0.95);

    const bool solved = cv::solvePnP(m_wandPoints,detPixels, m_camData.cameraMatrix,
                                     m_camData.distortionCoeffs, rVec, tVec, false);

    //    std::cout << "solved: " << solved << " for camera: " << camID << std::endl;

    if (!solved)
    {
        return std::nullopt;
    }
    /*
    std::cout << "rotation: " << rVec.at<double>(0) * 180.0/M_PI
              << " " << rVec.at<double>(1) * 180.0/M_PI
              << " " << rVec.at<double>(2) * 180.0/M_PI << std::endl;
    std::cout << "tvec: " << tVec.at<double>(0)
              << " " << tVec.at<double>(1)
              << " " << tVec.at<double>(2) << std::endl;
*/    
    cv::Mat rMat;
    cv::Rodrigues(rVec, rMat);

    return Eigen::Affine3f(cv::Affine3d(rMat, tVec).cast<float>());
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

std::vector<WandCalibration::ExtrGuessRes> WandCalibration::frameExtGuess(const QMap<int, std::vector<cv::Point2f>> &pixels)
{
    std::vector<WandCalibration::ExtrGuessRes> result;

    for (auto &cameraID : pixels.keys())
    {
        if (auto transform = relativeTransform(cameraID, pixels[cameraID]))
        {
            const float err = computeReprojectionError(m_camData, m_wandPoints, pixels[cameraID], transform.value());
            result.push_back({cameraID, err, transform});
        }
    }

    return result;
}

float WandCalibration::computeReprojectionError(const RPIMoCap::CameraParams &camData, const std::vector<cv::Point3f> wandPoints,
                                                const std::vector<cv::Point2f> &pixels, const Eigen::Affine3f &estTransform)
{
    std::vector<cv::Point3f> transformedWandPts;

    for (auto &wandPt : wandPoints)
    {
        Eigen::Vector3f tPt = estTransform * Eigen::Vector3f(wandPt.x, wandPt.y, wandPt.z);
        transformedWandPts.push_back(cv::Point3f(tPt.x(), tPt.y(), tPt.z()));
    }

    std::vector<cv::Point2f> projPixels;
    cv::projectPoints(transformedWandPts, cv::Vec3f(0,0,0), cv::Vec3f(0,0,0),
                      camData.cameraMatrix, camData.distortionCoeffs, projPixels);

    float error = 0.0f;

    assert(projPixels.size() == pixels.size());

    for (size_t i = 0; i < pixels.size(); ++i)
    {
        error += cv::norm(projPixels[i] - pixels[i]);
    }

    return error;
}

bool WandCalibration::haveAllExtrinsicGuess()
{
    for (auto &guess : m_extrinsicGuess)
    {
        if (!guess.transform.has_value())
        {
            return false;
        }
    }
    return true;
}
