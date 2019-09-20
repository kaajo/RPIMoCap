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

#include <eigen3/Eigen/Geometry>
#include <opencv2/core/types.hpp>

#include <msgpack.hpp>

namespace msgpack {
MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) {
namespace adaptor {

template <>
struct convert<Eigen::Vector3f> {
    msgpack::object const& operator()(msgpack::object const &o, Eigen::Vector3f &t) const {
        if (o.type != msgpack::type::ARRAY) throw msgpack::type_error();
        if (o.via.array.size != 3) throw msgpack::type_error();
        o.via.array.ptr[0] >> t.x();
        o.via.array.ptr[1] >> t.y();
        o.via.array.ptr[2] >> t.z();
        return o;
    }
};

template <>
struct pack<Eigen::Vector3f> {
    template <typename Stream>
    msgpack::packer<Stream>& operator()(msgpack::packer<Stream> &o, Eigen::Vector3f const &t) const {
        o.pack_array(3);
        o.pack(t.x());
        o.pack(t.y());
        o.pack(t.z());
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

template <>
struct convert<cv::Point2i> {
    msgpack::object const& operator()(msgpack::object const &o, cv::Point2i &t) const {
        if (o.type != msgpack::type::ARRAY) throw msgpack::type_error();
        if (o.via.array.size != 2) throw msgpack::type_error();
        o.via.array.ptr[0] >> t.x;
        o.via.array.ptr[1] >> t.y;
        return o;
    }
};

template <>
struct pack<cv::Point2i> {
    template <typename Stream>
    msgpack::packer<Stream>& operator()(msgpack::packer<Stream> &o, cv::Point2i const &t) const {
        o.pack_array(2);
        o.pack(t.x);
        o.pack(t.y);
        return o;
    }
};

} // namespace adaptor
} // MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
} // namespace msgpack
