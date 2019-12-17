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

class WandCalibration : public QObject
{
    Q_OBJECT
public:
    WandCalibration(QMap<int,std::shared_ptr<CameraSettings>> &cameraSettings,
                    RPIMoCap::CameraParams camData, QObject *parent = nullptr);

    void addFrame(const QMap<int, std::vector<cv::Point2f>> &points);
private:
    static std::optional<std::vector<cv::Point2f> > detect4pWand(const std::vector<cv::Point2f> &pts);

    enum class CalibrationState
    {
        Unknown = 0,
        ExtrinsicGuess = 10,
        Optimized = 20
    };

    struct CameraCalibState {
        int cameraID = -1;
        CalibrationState state = CalibrationState::Unknown;
        float reprojectionError = std::numeric_limits<float>::max();
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    };

    float computeReprojectionError(const std::vector<cv::Point2f> &pixels, const std::vector<cv::Point3f> &triangulatedPoints,
                                   const Eigen::Affine3f &estTransform);
    bool haveAllExtrinsicGuess();

    float computeScale(const cv::Mat &transformEstimation, std::vector<cv::Point3f> &triangulatedPoints);

    std::vector<cv::Point3f> m_wandPoints;

    int m_referenceCameraID = -1;

    QMap<int,std::shared_ptr<CameraSettings>> m_cameraSettings;
    QMap<int, CameraCalibState> m_extrinsicGuess;


    struct ObsDetection {
        std::vector<cv::Point2f> firstPixels;
        std::vector<cv::Point2f> secondPixels;
        float reprojectionError = std::numeric_limits<float>::max();
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    };

    QMap<std::pair<int,int>,ObsDetection> m_observedDetections;

    RPIMoCap::CameraParams m_camData;
};
