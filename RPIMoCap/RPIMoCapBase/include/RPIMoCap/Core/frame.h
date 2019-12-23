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
        Marker(){}
        Marker(size_t id, Eigen::Vector3f pos) : id(id), position(pos) {}

        size_t id = 0;
        Eigen::Vector3f position;

        MSGPACK_DEFINE_MAP(id,position);
    };

    Frame(const uint64_t time, const std::vector<Line3D> &lines)
        : m_time(time)
        , m_lines(lines)
    {
    }

    /*
    bool changeMarkerId(int oldID, int newID)
    {
        auto func = [](const int id, const Marker &marker){return id == marker.id();};

        auto point = std::find_if(m_markers.begin(), m_markers.end(), [func, oldID](const Marker &marker){return func(oldID, marker);});

        if(point == m_markers.end()) return false;

        auto pointWithSameID = std::find_if(m_markers.begin(), m_markers.end(), [func, newID](const Marker &marker){return func(newID, marker);});

        if(pointWithSameID != m_markers.end())
        {
            ++m_maximalPointID;

            changeMarkerId(newID, m_maximalPointID);
        }

        point->setId(newID);
        return true;
    }
    */

    std::vector<Line3D> lines() const;

    std::vector<Marker> markers() const;
    void setMarkers(const std::vector<Marker> &markers);

private:
    uint64_t m_time;
    std::vector<Marker> m_markers;
    std::vector<Line3D> m_lines;

    MSGPACK_DEFINE_MAP(m_time,m_markers,m_lines);
};

}
