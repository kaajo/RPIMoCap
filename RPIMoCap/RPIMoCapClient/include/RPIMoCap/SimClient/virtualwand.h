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

#include "simmarker.h"

#include <RPIMoCap/Core/frame.h>

#include <Eigen/Geometry>

namespace RPIMoCap::SimClient {

class VirtualWand
{
public:
    VirtualWand(float sizecm, float middlePointOffsetcm);

    std::vector<SimMarker> markers(const Eigen::Affine3f &transform) const;

    static constexpr size_t wandPointCount = 3;

private:
    Eigen::Vector3f m_leftPoint;
    Eigen::Vector3f m_middlePoint;
    Eigen::Vector3f m_rightPoint;
};

class VirtualFloorWand
{
public:
    VirtualFloorWand(float sizecm);

    std::vector<SimMarker> markers(const Eigen::Affine3f &transform) const;

private:
    Eigen::Vector3f m_centerPoint;
    Eigen::Vector3f m_leftPoint;
    Eigen::Vector3f m_rightPoint;
    Eigen::Vector3f m_nearPoint;
    Eigen::Vector3f m_farPoint;
};

}
