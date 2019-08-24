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

#include "markerdetector.h"

#include <QtConcurrent/QtConcurrentFilter>
#include <QtConcurrent/QtConcurrentMap>
#include <QMatrix4x4>

#include <functional>

MarkerDetector::MarkerDetector(cv::Size2f cameraFoVRad)
    : m_cameraFoVRad(cameraFoVRad)
{

}

cv::Mat MarkerDetector::computePixelDirs(const cv::Size2i &resolution, cv::Size2f cameraFoVRad)
{
    const float tanHorizontal_ = std::tan(cameraFoVRad.width / 2);
    const float tanVertical_ = std::tan(cameraFoVRad.height / 2);

    cv::Mat pixelLines(resolution, CV_32FC3);

    for(int row = 0; row < resolution.height; ++row)
    {
        for(int col = 0; col < resolution.width ; ++col)
        {
            /*
            const Eigen::Vector2f distanceFromCenter(i - resolution.width/2.0f,j - resolution.height/2.0f);

            QMatrix4x4 rotMatrix;
            rotMatrix.rotate(distanceFromCenter.x() * horizontalFov/resolution.width, 0, 1, 0);

            QMatrix4x4 rotMatrix2;
            rotMatrix2.rotate(distanceFromCenter.y() * horizontalFov/resolution.width, 1,0,0);

            const QVector3D partialRes = rotMatrix * QVector3D(0.0,0.0,1.0);
            const QVector3D dir = rotMatrix2 * partialRes;
            pixelLines.at<cv::Vec3f>(cv::Point(j,i)) = cv::Vec3f(dir.x(),dir.y(),dir.z());
            */

            const float directionX = (2 * ((col + 0.5f) / resolution.width) - 1) * tanHorizontal_;
            const float directionY = (1 - 2 * (row + 0.5f) / resolution.height) * tanVertical_;
            const auto dir = Eigen::Vector3f(directionX, directionY, 1).normalized();
            pixelLines.at<cv::Vec3f>(cv::Point(col,row)) = cv::Vec3f(dir.x(), dir.y(), dir.z());
        }
    }

    return pixelLines;
}

Eigen::Vector3f MarkerDetector::computePixelDir(const cv::Size2i &resolution, cv::Size2f cameraFoVRad, cv::Point2i pixel)
{
    const float tanHorizontal_ = std::tan(cameraFoVRad.width / 2);
    const float tanVertical_ = std::tan(cameraFoVRad.height / 2);

    float directionX = (2 * ((pixel.x + 0.5f) / resolution.width) - 1) * tanHorizontal_;
    float directionY = (1 - 2 * (pixel.y + 0.5f) / resolution.height) * tanVertical_;
    return Eigen::Vector3f(directionX, directionY, 1).normalized();

    /*
    const Eigen::Vector2f distanceFromCenter(row - resolution.width/2,col - resolution.height/2);

    QMatrix4x4 rotMatrix;
    rotMatrix.rotate(distanceFromCenter.x() * fovHRad, 0, 1, 0);

    QMatrix4x4 rotMatrix2;
    rotMatrix2.rotate(distanceFromCenter.y() * fovHRad, 1,0,0);

    const QVector3D partialRes = rotMatrix * QVector3D(0.0,0.0,1.0);
    const QVector3D dir = rotMatrix2 * partialRes;
    return cv::Vec3f(dir.x(),dir.y(),dir.z());
    */
}

/*
Eigen::ParametrizedLine<double, 3>
RoboTrain::Utilities::FrustumFactory::pixelLineFromDirection(int x, int y) const {
    auto newMatrix =
            cv::getOptimalNewCameraMatrix(intrinsicMatrix_, distortionParams_, cv::Size(imageWidth_, imageHeight_), 1);

    std::array<cv::Point2f, 1> points = {cv::Point2f(x, y)};
    std::array<cv::Point2f, 1> undistortedPoints;
    cv::undistortPoints(points, undistortedPoints, intrinsicMatrix_, distortionParams_, newMatrix);

    float directionX = (2 * ((undistortedPoints[0].x + 0.5) / imageWidth_) - 1) * tanHorizontal_;
    float directionY = (1 - 2 * (undistortedPoints[0].y + 0.5) / imageHeight_) * tanVertical_;
    return {Eigen::Vector3d(), Eigen::Vector3d(directionX, directionY, 1).normalized()};
}
*/

std::vector<RPIMoCap::Line3D> MarkerDetector::onImage(const cv::Mat &image)
{
    if (m_pixelLines.empty()) {
        m_pixelLines = computePixelDirs(cv::Size2i(image.cols,image.rows) ,m_cameraFoVRad);
        qDebug() << "compute lines: " << image.cols << " " << image.rows;
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

    const std::vector<RPIMoCap::Line3D> lines = QtConcurrent::blockingMapped<std::vector<RPIMoCap::Line3D>>(contours,
              std::bind(&MarkerDetector::qtConcurrentpickLine, this, std::placeholders::_1));

    return lines;
}

RPIMoCap::Line3D MarkerDetector::qtConcurrentpickLine(const std::vector<cv::Point2i> &contour)
{
    const int m00 = contour.size();
    const cv::Point2i sum = std::accumulate(contour.begin(),contour.end(),cv::Point2i(0,0));

    const int x = std::round(sum.x/static_cast<double>(m00));
    const int y = std::round(sum.y/static_cast<double>(m00));

    return RPIMoCap::Line3D({0.0,0.0,0.0}, m_pixelLines.at<Eigen::Vector3f>(cv::Point2i(x,y)));
}

