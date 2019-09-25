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

#include <RPIMoCap/Core/cameraparams.h>

#include <opencv2/core/mat.hpp>
#include <eigen3/Eigen/Geometry>

#include <mutex>

namespace RPIMoCap::SimClient {

class SimScene
{
public:
    struct Marker
    {
        size_t id;
        uint8_t sizemm;
        cv::Point3f translation;
    };

    SimScene() = default;

    void setMarkers(const std::vector<Marker> markers);
    cv::Mat projectScene(const CameraParams &params) const;

private:
    mutable std::mutex m_dataMutex;
    std::vector<Marker> m_markers;
};

}
