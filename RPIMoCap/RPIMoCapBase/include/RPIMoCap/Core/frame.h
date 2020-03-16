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

#include "msgpack_defs.h"
#include "line3d.h"

#include <Eigen/Geometry>
#include <msgpack/type.hpp>
#include <msgpack.hpp>

#include <chrono>

namespace RPIMoCap {

/**
 * @brief Frame contains all information captured in set of images from synchronized cameras.
 */
class Frame
{
public:
    struct Marker
    {
        size_t id = 0;
        Eigen::Vector3f position;

        MSGPACK_DEFINE_MAP(id,position);
    };

    struct LineSegment
    {
        float lengthcm = 100.0f;
        Line3D line;

        MSGPACK_DEFINE_MAP(lengthcm, line);
    };

    Frame(const std::chrono::high_resolution_clock::time_point time,
          const std::vector<LineSegment> &lines);

    std::vector<LineSegment> lines() const;

    std::vector<Marker> markers() const;
    void setMarkers(const std::vector<Marker> &markers);

private:
    std::chrono::high_resolution_clock::time_point m_time;
    std::vector<Marker> m_markers;
    std::vector<LineSegment> m_lines;

    MSGPACK_DEFINE_MAP(m_time,m_markers,m_lines);
};

}
