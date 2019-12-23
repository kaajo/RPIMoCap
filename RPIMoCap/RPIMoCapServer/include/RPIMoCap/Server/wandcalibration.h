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

#pragma once

#include <RPIMoCap/Core/cameraparams.h>
#include <RPIMoCap/Server/camerasettings.h>
#include <RPIMoCap/Core/msgpack_defs.h>

#include <QObject>
#include <QMap>

#include <opencv2/core/mat.hpp>

//TODO remove
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>

class WandCalibration : public QObject
{
    Q_OBJECT
public:
    WandCalibration(QMap<int,std::shared_ptr<CameraSettings>> &cameraSettings,
                    RPIMoCap::CameraParams camData, QObject *parent = nullptr);

    void addFrame(const QMap<int, std::vector<cv::Point2f>> &points);
private:
    static std::optional<std::vector<cv::Point2f> > detect4pWand(const std::vector<cv::Point2f> &pts);
    static std::optional<std::vector<cv::Point2f> > detect3pWand(const std::vector<cv::Point2f> &pts);

    float computeReprojectionError(const std::vector<cv::Point2f> &pixels, const std::vector<cv::Point3d> &triangulatedPoints,
                                   const Eigen::Affine3f &estTransform);

    float computeScale(const cv::Mat &transformEstimation, std::vector<cv::Point3d> &triangulatedPoints);

    std::vector<cv::Point3f> m_wandPoints;

    QMap<int,std::shared_ptr<CameraSettings>> m_cameraSettings;
    //QMap<int, CameraCalibState> m_extrinsicGuess;

    bool finished = false;

    float m_errorTreshold = 2.0f;

    struct ObsDetection {
        std::vector<cv::Point2f> firstPixels;
        std::vector<cv::Point2f> secondPixels;
        std::vector<cv::Point3d> triangulatedPoints;
        float reprojectionError = std::numeric_limits<float>::max();
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();

        std::array<double,9> cameraFirstData;
        double* cameraFirstParams(double focalLength)
        {
            cameraFirstData[0] = 0.0;
            cameraFirstData[1] = 0.0;
            cameraFirstData[2] = 0.0;
            cameraFirstData[3] = 0.0;
            cameraFirstData[4] = 0.0;
            cameraFirstData[5] = 0.0;

            return &cameraFirstData[0];
        }

        std::array<double,9> cameraSecondData;
        double* cameraSecondParams(double focalLength)
        {
            cv::Mat secondTransform;
            cv::eigen2cv(transform.matrix(), secondTransform);
            secondTransform.convertTo(secondTransform, CV_64FC1);

            std::cout << secondTransform << std::endl;

            cv::Mat rotVec;
            cv::Rodrigues(secondTransform, rotVec);

            cameraSecondData[0] = rotVec.at<double>(0);
            cameraSecondData[1] = rotVec.at<double>(1);
            cameraSecondData[2] = rotVec.at<double>(2);
            cameraSecondData[3] = transform.translation().x();
            cameraSecondData[4] = transform.translation().y();
            cameraSecondData[5] = transform.translation().z();

            return &cameraSecondData[0];
        }

    };

    QMap<std::pair<int,int>,ObsDetection> m_observedDetections;

    RPIMoCap::CameraParams m_camData;
};
