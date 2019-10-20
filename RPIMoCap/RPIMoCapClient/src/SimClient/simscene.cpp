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

#include <opencv2/opencv.hpp>

#include <QDebug>

namespace RPIMoCap::SimClient {

void SimScene::setMarkers(const std::vector<SimMarker> markers)
{
    std::scoped_lock lock(m_dataMutex);
    m_markers = markers;
}

cv::Mat SimScene::projectScene(const CameraParams &params) const
{
    std::vector<cv::Vec3f> pts;

    {
        std::scoped_lock lock(m_dataMutex);

        std::transform(m_markers.begin(), m_markers.end(), std::back_inserter(pts),
                       [](auto &marker){return marker.translation;});
    }

    cv::Mat simulatedImage(params.imageSize, CV_8UC1, cv::Scalar(0));

    if (pts.empty()) {
        return simulatedImage;
    }

    std::vector<cv::Point2f> pixels;
    cv::projectPoints(pts, params.rotation, params.translation,
                      params.cameraMatrix, params.distortionCoeffs, pixels);

    for (auto &px : pixels)
    {
        cv::circle(simulatedImage, px, 5, cv::Scalar(255), -1);
    }

    return simulatedImage;
}


}
