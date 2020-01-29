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

#include <RPIMoCap/SimClient/simscene.h>

#include <opencv2/core/affine.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <QDebug>
#include <iostream>

namespace RPIMoCap::SimClient {

void SimScene::setMarkers(const std::vector<SimMarker> markers)
{
    std::scoped_lock lock(m_dataMutex);
    m_markers = markers;
}

cv::Mat SimScene::projectScene(const CameraParams &params) const
{
    const cv::Affine3d cameraTransform(params.rotation, params.translation);

    std::vector<cv::Vec3f> pointsInCamera;

    {
        std::scoped_lock lock(m_dataMutex);

        std::transform(m_markers.begin(), m_markers.end(), std::back_inserter(pointsInCamera),
                       [cameraTransform](auto &marker)
                       {return cameraTransform.inv() * marker.translation;});
    }

    cv::Mat simulatedImage(params.imageSize, CV_8UC1, cv::Scalar(0));

    if (pointsInCamera.empty()) {
        return simulatedImage;
    }

    for (auto &pt : pointsInCamera)
    {
        std::cout << "point: " << pt << std::endl;
    }

    std::cout << params.translation << std::endl;
    std::cout << params.rotation << std::endl;

    std::vector<cv::Point2f> pixels;
    cv::projectPoints(pointsInCamera, cv::Vec3f::zeros(), cv::Vec3f::zeros(),
                      params.cameraMatrix, params.distortionCoeffs, pixels);

    for (auto &pt : pixels)
    {
        std::cout << "pixel: " << pt << std::endl;
    }

    for (size_t i = 0; i < pixels.size(); ++i)
    {
        if (pointsInCamera[i][2] > 0.0f)
        {
            cv::circle(simulatedImage, pixels[i], 3, cv::Scalar(255), -1);
        }
    }

    return simulatedImage;
}

}
