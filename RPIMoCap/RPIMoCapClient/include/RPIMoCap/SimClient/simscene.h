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

#include "RPIMoCap/SimClient/simmarker.h"

#include <RPIMoCap/Core/cameraparams.h>

#include <opencv2/core/mat.hpp>

#include <mutex>

namespace RPIMoCap::SimClient {

class SimScene
{
public:
    /**
     * @brief setMarkers
     * @param markers Markers in absolute 3D position.
     */
    void setMarkers(const std::vector<SimMarker> &markers);

    /**
     * @brief projectScene is function used for Camera simulation.
     * @param params All intrinsic parameters
     * @param rVec Rotation vector
     * @param tVec Translation vector
     * @return Simulated camera image with projected markers.
     */
    cv::Mat projectScene(const Camera::Intrinsics &params,
                         const cv::Vec3f &rVec, const cv::Vec3f &tVec) const;

private:
    mutable std::mutex m_dataMutex;
    std::vector<SimMarker> m_markers;
};

}
