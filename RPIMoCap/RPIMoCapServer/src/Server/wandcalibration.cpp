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
        m_extrinsicGuess[camSettings->id()] = std::nullopt;
    }

    m_wandPoints.push_back({-50.0,0.0,0.0});
    m_wandPoints.push_back({20.0,0.0,0.0});
    m_wandPoints.push_back({50.0,0.0,0.0});
    m_wandPoints.push_back({20.0,10.0,0.0});
}

void WandCalibration::addFrame(const QMap<int, std::vector<cv::Point2f> > &points)
{
    m_observations.push_back(points);

    for (auto &key : points.keys())
    {
        auto transform = extrinsicGuess(key, points[key]);
        
        if (transform.has_value())
        {
            m_extrinsicGuess[key] = transform;
        }
    }

    if (haveAllExtrinsicGuess())
    {
        const auto keys = m_extrinsicGuess.keys();

        std::cout << "not normalised extrinsic guess:" << std::endl;
        for (auto &guessKey : keys)
        {
            std::cout << m_extrinsicGuess[guessKey].value().matrix() << std::endl;
        }

        //change transform relative to first one

        m_cameraSettings[keys[0]]->setTransform(Eigen::Affine3f::Identity());

        const Eigen::Affine3f firstTf = m_extrinsicGuess[keys[0]].value();

        for (size_t i = 1; i < keys.size(); ++i)
        {
            m_cameraSettings[keys[i]]->setTransform(firstTf * m_extrinsicGuess[keys[i]].value().inverse());
        }
    }
}

std::optional<Eigen::Affine3f> WandCalibration::extrinsicGuess(const size_t camID, const std::vector<cv::Point2f> &pixels)
{
    if (pixels.size() != m_wandPoints.size())
    {
        return std::nullopt;
    }

    const std::vector<cv::Point2f> detPixels = detect4pWand(pixels);

    cv::Mat rVec;
    cv::Mat tVec;

    const bool solved = cv::solvePnPRansac(m_wandPoints,detPixels, m_camData.cameraMatrix,
                                     m_camData.distortionCoeffs, rVec, tVec, false, 300, 8.0, 0.95);

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

    return Eigen::Affine3d(cv::Affine3d(rMat, tVec)).cast<float>();
}

std::vector<cv::Point2f> WandCalibration::detect4pWand(const std::vector<cv::Point2f> &pts)
{
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

    return {pts[borderL], pts[middle], pts[borderR], pts[cross]};
}

bool WandCalibration::haveAllExtrinsicGuess()
{
    for (auto &guess : m_extrinsicGuess)
    {
        if (!guess.has_value())
        {
            return false;
        }
    }
    return true;
}
