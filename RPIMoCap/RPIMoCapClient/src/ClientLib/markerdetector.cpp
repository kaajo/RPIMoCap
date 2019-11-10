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

#include "RPIMoCap/ClientLib/markerdetector.h"

#include <QtConcurrent/QtConcurrentFilter>
#include <QtConcurrent/QtConcurrentMap>
#include <QMatrix4x4>

#include <opencv2/imgproc.hpp>

#include <functional>

MarkerDetector::MarkerDetector(const RPIMoCap::CameraParams &camParams)
    : m_camParams(camParams)
{

}

cv::Mat MarkerDetector::computePixelDirs(const RPIMoCap::CameraParams &camParams)
{
    cv::Mat pixelLines(camParams.imageSize, CV_32FC3);

    for(int row = 0; row < camParams.imageSize.height; ++row)
    {
        for(int col = 0; col < camParams.imageSize.width ; ++col)
        {
            const auto v = computePixelDir(camParams.cameraMatrix, camParams.distortionCoeffs, cv::Point2i(col,row));
            pixelLines.at<cv::Point3f>(row, col) = cv::Point3f(v.x(), v.y(), v.z());
        }
    }

    return pixelLines;
}

Eigen::Vector3f MarkerDetector::computePixelDir(cv::Mat cameraMatrix, cv::Mat distortionCoeffs, cv::Point2i pixel)
{
    std::vector<cv::Point2f> points = {pixel};
    cv::Mat undistortedPoints;
    cv::undistortPoints(points, undistortedPoints, cameraMatrix, distortionCoeffs);
    return Eigen::Vector3f(undistortedPoints.at<float>(0,0), undistortedPoints.at<float>(0,1), 1).normalized();
}

void MarkerDetector::onImage(const cv::Mat &image, std::vector<RPIMoCap::Line3D> &lines, std::vector<cv::Point2i> &points)
{
    lines.clear();
    points.clear();

    if (m_pixelLines.empty()) {
        m_pixelLines = computePixelDirs(m_camParams);
        qDebug() << "compute lines: " << image.cols << "x" << image.rows;
    }

    cv::Mat filterImage;

    //cv::cvtColor(image, filterImage, cv::COLOR_BGR2GRAY);
    //cv::medianBlur(filterImage, filterImage, 3);

    cv::threshold(image, filterImage, 220, 255, cv::THRESH_BINARY);

    // cv::morphologyEx(filterImage, filterImage, cv::MORPH_OPEN , dilateKernel);

    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(filterImage, contours , cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    QtConcurrent::blockingFilter(contours, [](const std::vector<cv::Point> &contour)
    {
        ///if area of component is too big, it is probably some big light or window
        ///if                      too small, it is noise
        double contArea = contour.size();
        return contArea < 500 && contArea > 5;
    });

    points = QtConcurrent::blockingMapped<std::vector<cv::Point2i>>(contours,
                                                                    std::bind(&MarkerDetector::qtConcurrentfindPoint, this, std::placeholders::_1));
    lines = QtConcurrent::blockingMapped<std::vector<RPIMoCap::Line3D>>(contours,
                                                                        std::bind(&MarkerDetector::qtConcurrentpickLine, this, std::placeholders::_1));
}

cv::Point2i MarkerDetector::qtConcurrentfindPoint(const std::vector<cv::Point2i> &contour)
{
    const int m00 = contour.size();
    const cv::Point2i sum = std::accumulate(contour.begin(),contour.end(),cv::Point2i(0,0));

    const int x = std::round(sum.x/static_cast<double>(m00));
    const int y = std::round(sum.y/static_cast<double>(m00));

    return cv::Point2i(x, y);
}

RPIMoCap::Line3D MarkerDetector::qtConcurrentpickLine(const std::vector<cv::Point2i> &contour)
{
    const int m00 = contour.size();
    const cv::Point2i sum = std::accumulate(contour.begin(),contour.end(),cv::Point2i(0,0));

    const int x = std::round(sum.x/static_cast<double>(m00));
    const int y = std::round(sum.y/static_cast<double>(m00));

    return RPIMoCap::Line3D({0.0,0.0,0.0}, m_pixelLines.at<Eigen::Vector3f>(cv::Point2i(x,y)));
}

