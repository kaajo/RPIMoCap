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

#include <QUuid>

#include <Eigen/Geometry>
#include <msgpack/type.hpp>
#include <msgpack.hpp>

#include <chrono>

// needed because https://bugreports.qt.io/browse/QTBUG-69701
namespace std
{
template<> struct hash<QUuid>
{
    std::size_t operator()(const QUuid& s) const noexcept
    {
        return qHash(s);
    }
};
}

namespace RPIMoCap {


/**
 * @brief Frame contains all information captured in set of images from synchronized cameras.
 */
class Frame
{
public:
    using Time = std::chrono::high_resolution_clock::time_point;

    struct CamObservation {
        size_t targetId = std::numeric_limits<size_t>::quiet_NaN();
        cv::Point2f px;
        Line3D ray;

        MSGPACK_DEFINE_MAP(targetId,px,ray);
    };
    using CamObservations = std::vector<CamObservation>;

    struct Marker
    {
        size_t id = std::numeric_limits<size_t>::quiet_NaN();
        std::unordered_map<QUuid, CamObservations> observations;
        Eigen::Vector3f position;

        MSGPACK_DEFINE_MAP(id,position);
    };

    Frame(const Time time = Time::min(), const std::unordered_map<QUuid, CamObservations> &obs = {});

    auto observations() const {
        return m_observations;
    }

    std::vector<Marker> markers() const;
    void setMarkers(const std::vector<Marker> &markers);

private:
    Time m_time;

    std::unordered_map<QUuid, CamObservations> m_observations;

    std::vector<Marker> m_markers;

    MSGPACK_DEFINE_MAP(m_time,m_observations, m_markers);
};

}
