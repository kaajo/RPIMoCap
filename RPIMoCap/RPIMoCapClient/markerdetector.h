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

#include <RPIMoCapBase/line3d.h>
#include <RPIMoCapBase/msgpack_defs.h>

#include <QObject>

#include <opencv2/opencv.hpp>

class MarkerDetector
{
public:
    explicit MarkerDetector(cv::Size2f cameraFoVRad);

    static cv::Mat computePixelDirs(const cv::Size2i &resolution, cv::Size2f cameraFoVRad);
    static Eigen::Vector3f computePixelDir(const cv::Size2i &resolution, cv::Size2f cameraFoVRad, cv::Point2i pixel);

    void onImage(const cv::Mat &image, std::vector<RPIMoCap::Line3D> &lines, std::vector<cv::Point2i> &points);

private:
    RPIMoCap::Line3D qtConcurrentpickLine(const std::vector<cv::Point2i> &contour);

    cv::Size2f m_cameraFoVRad;
    cv::Mat m_pixelLines;
    //cv::Mat dilateKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
};
