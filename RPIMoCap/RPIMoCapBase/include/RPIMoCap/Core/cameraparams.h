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

#include <opencv2/core/mat.hpp>

namespace RPIMoCap {

struct CameraParams
{
    cv::Size imageSize = cv::Size(0,0);
    cv::Vec3f translation = cv::Vec3f(0.0, 0.0, 0.0);
    cv::Vec3f rotation = cv::Vec3f(0.0, 0.0, 0.0);
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_32FC1);
    cv::Mat distortionCoeffs = cv::Mat(1, 4, CV_32FC1, cv::Scalar(0.0f));
};

}
