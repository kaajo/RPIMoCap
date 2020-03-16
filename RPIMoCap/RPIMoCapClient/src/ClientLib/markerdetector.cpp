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

#include <QtConcurrent/QtConcurrentMap>

#include <opencv2/imgproc.hpp>

#include <functional>

MarkerDetector::MarkerDetector(const Params &algParams)
    : m_algParams(algParams)
{

}

//cv::Mat MarkerDetector::computePixelDirs(const RPIMoCap::CameraParams &camParams)
//{
//    cv::Mat pixelLines(camParams.imageSize, CV_32FC3);

//for(int row = 0; row < camParams.imageSize.height; ++row)
//{
//    for(int col = 0; col < camParams.imageSize.width ; ++col)
//    {
//        const auto v = computePixelDir(camParams.cameraMatrix, camParams.distortionCoeffs, cv::Point2i(col,row));
//        pixelLines.at<cv::Point3f>(row, col) = cv::Point3f(v.x(), v.y(), v.z());
//    }
//}

//return pixelLines;
//}

//Eigen::Vector3f MarkerDetector::computePixelDir(cv::Mat cameraMatrix, cv::Mat distortionCoeffs, cv::Point2f pixel)
//{
//    std::vector<cv::Point2f> points = {pixel};
//    cv::Mat undistortedPoints;
//    cv::undistortPoints(points, undistortedPoints, cameraMatrix, distortionCoeffs);
//    return Eigen::Vector3f(undistortedPoints.at<float>(0,0), undistortedPoints.at<float>(0,1), 1).normalized();
//}

std::vector<cv::Point2f> MarkerDetector::detectMarkers(const cv::Mat &image)
{
    cv::Mat filterImage;

    cv::threshold(image, filterImage, m_algParams.imageThreshold, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(filterImage, contours , cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    contours.erase(std::remove_if(contours.begin(), contours.end(), [p=m_algParams](const std::vector<cv::Point> &contour)
                                  {return contour.size() > p.maxContourSize &&
                                           contour.size() < p.minContourSize;}), contours.end());

    return QtConcurrent::blockingMapped<std::vector<cv::Point2f>>(contours, &MarkerDetector::findPoint);
}

cv::Point2f MarkerDetector::findPoint(const std::vector<cv::Point2i> &contour)
{
    const int m00 = contour.size();
    const cv::Point2i sum = std::accumulate(contour.begin(),contour.end(),cv::Point2i(0,0));

    const float x = static_cast<float>(sum.x)/static_cast<float>(m00);
    const float y = static_cast<float>(sum.y)/static_cast<float>(m00);

    return cv::Point2f(x, y);
}
