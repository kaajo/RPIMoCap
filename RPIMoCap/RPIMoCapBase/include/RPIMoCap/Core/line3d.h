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

#include <Eigen/Geometry>

#include <msgpack.hpp>

namespace RPIMoCap {

/**
 * @brief The Line class represents parametric line/ray in 3D space.
 */
using Line3D = Eigen::ParametrizedLine<float,3>;

/**
* @brief Finds closest points on two rays.
* @param[in] line1 First line
* @param[in] line2 Second line
* @param[out] closestPointLine1 Point on line1
* @param[out] closestPointLine2 Point on line2
* @return If closest point lies on ray returns false, otherwise true.
*/
bool closestPoints(const Line3D &line1,const Line3D &line2, Eigen::Vector3f &pointLine1, Eigen::Vector3f &pointLine2);

/**
* @brief Angle between two 3D vectors.
* @param v1 First vector
* @param v2 Second vector
* @return Angle in radians
*/
float lineAngle(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2);

} //namespace RPIMoCap

namespace msgpack {
MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) {
namespace adaptor {

template <>
struct convert<RPIMoCap::Line3D> {
    msgpack::object const& operator()(msgpack::object const &o, RPIMoCap::Line3D &t) const {
        if (o.type != msgpack::type::ARRAY) throw msgpack::type_error();
        if (o.via.array.size != 6) throw msgpack::type_error();
        //origin
        o.via.array.ptr[0] >> t.origin().x();
        o.via.array.ptr[1] >> t.origin().y();
        o.via.array.ptr[2] >> t.origin().z();
        //direction
        o.via.array.ptr[3] >> t.direction().x();
        o.via.array.ptr[4] >> t.direction().y();
        o.via.array.ptr[5] >> t.direction().z();
        return o;
    }
};

template <>
struct pack<RPIMoCap::Line3D> {
    template <typename Stream>
    msgpack::packer<Stream>& operator()(msgpack::packer<Stream> &o, RPIMoCap::Line3D const &t) const {
        o.pack_array(6);
        o.pack(t.origin().x());
        o.pack(t.origin().y());
        o.pack(t.origin().z());
        o.pack(t.direction().x());
        o.pack(t.direction().y());
        o.pack(t.direction().z());

        return o;
    }
};

/*
template <>
struct object_with_zone<my_class> {
    void operator()(msgpack::object::with_zone& o, my_class const& v) const {
        o.type = type::ARRAY;
        o.via.array.size = 2;
        o.via.array.ptr = static_cast<msgpack::object*>(
            o.zone.allocate_align(sizeof(msgpack::object) * o.via.array.size));
        o.via.array.ptr[0] = msgpack::object(v.get_name(), o.zone);
        o.via.array.ptr[1] = msgpack::object(v.get_age(), o.zone);
    }
};
*/

} // namespace adaptor
} // MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
} // namespace msgpack
