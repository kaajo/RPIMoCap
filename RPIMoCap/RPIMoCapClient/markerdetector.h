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

#include <line3d.h>

#include <QObject>

#include <opencv2/opencv.hpp>

class MarkerDetector
{
public:
    explicit MarkerDetector();

    static cv::Mat computePixelLines(const cv::Size2i &resolution, const float horizontalFov);
    static cv::Vec3f computePixelLine(const cv::Size2i &resolution, const float degreesPerPixel, const int row, const int col);

    std::vector<RPIMoCap::Line3D> onImage(const cv::Mat &image);

private:
    RPIMoCap::Line3D qtConcurrentpickLine(const std::vector<cv::Point> &contour);

    inline Eigen::Vector3f pixelLineDirectionVector(const int x,const int y)
    {
        auto vec = m_pixelLines.at<cv::Vec3d>(cv::Point2d(x,y));
        return Eigen::Vector3f(vec[0],vec[1],vec[2]);
    }
    cv::Mat m_pixelLines;
    cv::Mat dilateKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
};
