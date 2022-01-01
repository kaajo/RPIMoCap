/*
 * This file is part of the RPIMoCap (https://github.com/kaajo/RPIMoCap).
 * Copyright (c) 2021 Miroslav Krajicek.
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

#include <Server/camerasettings.h>
#include <Server/wanddetector.h>

#include <RPIMoCap/Core/cameraparams.h>

namespace RPIMoCap {

class FloorCalibration {

public:
    struct CameraData {
        QUuid id;
        RPIMoCap::Camera::Intrinsics intrinsics;
        // TODO ?? Eigen::Affine3f extrinsics;
        std::vector<cv::Point2f> detectedPixels;
        std::vector<Line3D> rays;
    };

    static std::optional<Eigen::Affine3f> computeOrigin(std::vector<CameraData>  data, float size);

private:
    static std::optional<float> averageDistance(std::vector<Line3D> rays1, std::vector<Line3D> rays2,
                                                const Eigen::Vector3f &centerPoint, const float distanceThreshold);

    static std::vector<Line3D> createRaysFromIDs(const CameraData &camData, const std::vector<size_t> &ids);

    static Eigen::Vector3f computeNormalVector(const std::vector<Eigen::Vector3f>& points);

    static std::optional<std::array<Line3D, 4>> findXZAxes(const std::vector<Eigen::Vector3f>& crossPoints);

};

}
