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

#include <opencv2/core/types.hpp>

class MarkerDetector
{
public:
    struct Params
    {
        int imageThreshold = 220;
        size_t minContourSize = 5;
        size_t maxContourSize = 500;
    };

    explicit MarkerDetector(const Params &algParams);

//    static cv::Mat computePixelDirs(const RPIMoCap::CameraParams &camParams);
//    static Eigen::Vector3f computePixelDir(cv::Mat cameraMatrix, cv::Mat distortionCoeffs, cv::Point2f pixel);

    std::vector<cv::Point2f> detectMarkers(const cv::Mat &image);

private:
    static cv::Point2f findPoint(const std::vector<cv::Point2i> &contour);

    Params m_algParams;
};
