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

MarkerDetector::MarkerDetector()
{

}

cv::Mat MarkerDetector::computePixelLines(const cv::Size2i &resolution, const float horizontalFov)
{
    cv::Mat pixelLines(resolution, CV_32FC3);

    for(int i = 0; i < resolution.height; ++i)
    {
        for(int j = 0; j < resolution.width ; j++)
        {
            const Eigen::Vector2f distanceFromCenter(i - resolution.width/2.0f,j - resolution.height/2.0f);

            QMatrix4x4 rotMatrix;
            rotMatrix.rotate(distanceFromCenter.x() * horizontalFov/resolution.width, 0, 1, 0);

            QMatrix4x4 rotMatrix2;
            rotMatrix2.rotate(distanceFromCenter.y() * horizontalFov/resolution.width, 1,0,0);

            const QVector3D partialRes = rotMatrix * QVector3D(0.0,0.0,1.0);
            const QVector3D dir = rotMatrix2 * partialRes;
            pixelLines.at<cv::Vec3f>(cv::Point(j,i)) = cv::Vec3f(dir.x(),dir.y(),dir.z());
        }
    }

    return pixelLines;
}

cv::Vec3f MarkerDetector::computePixelLine(const cv::Size2i &resolution, const float degreesPerPixel, const int row, const int col)
{
    const Eigen::Vector2f distanceFromCenter(row - resolution.width/2,col - resolution.height/2);

    QMatrix4x4 rotMatrix;
    rotMatrix.rotate(distanceFromCenter.x() * degreesPerPixel, 0, 1, 0);

    QMatrix4x4 rotMatrix2;
    rotMatrix2.rotate(distanceFromCenter.y() * degreesPerPixel, 1,0,0);

    const QVector3D partialRes = rotMatrix * QVector3D(0.0,0.0,1.0);
    const QVector3D dir = rotMatrix2 * partialRes;
    return cv::Vec3f(dir.x(),dir.y(),dir.z());
}

std::vector<RPIMoCap::Line3D> MarkerDetector::onImage(const cv::Mat &image)
{
    if (m_pixelLines.empty()) {
        m_pixelLines = computePixelLines(cv::Size2i(image.cols,image.rows) ,60);
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

    const QVector<RPIMoCap::Line3D> lines = QtConcurrent::blockingMapped<QVector<RPIMoCap::Line3D>>(contours,
              std::bind(&MarkerDetector::qtConcurrentpickLine, this, std::placeholders::_1));

    //const QVector<RPIMoCap::Line3D> lines;

    return lines.toStdVector();
}

RPIMoCap::Line3D MarkerDetector::qtConcurrentpickLine(const std::vector<cv::Point> &contour)
{
    double m00 = contour.size(), m10 = 0.0 , m01 = 0.0;

    for(const cv::Point &pnt : contour)
    {
        m10 += pnt.x;
        m01 += pnt.y;
    }

    const int x = m10/m00;
    const int y = m01/m00;

    return RPIMoCap::Line3D({0.0,0.0,0.0},pixelLineDirectionVector(x,y));
}

