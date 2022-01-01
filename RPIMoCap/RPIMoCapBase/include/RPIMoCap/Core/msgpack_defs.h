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
#include <opencv2/core/types.hpp>
#include <msgpack.hpp>
#include <QUuid>

#include <math.h>
#include <chrono>

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
struct convert<cv::Point2f> {
    msgpack::object const& operator()(msgpack::object const &o, cv::Point2f &t) const {
        if (o.type != msgpack::type::ARRAY) throw msgpack::type_error();
        if (o.via.array.size != 2) throw msgpack::type_error();
        o.via.array.ptr[0] >> t.x;
        o.via.array.ptr[1] >> t.y;
        return o;
    }
};

template <>
struct pack<cv::Point2f> {
    template <typename Stream>
    msgpack::packer<Stream>& operator()(msgpack::packer<Stream> &o, cv::Point2f const &t) const {
        o.pack_array(2);
        o.pack(t.x);
        o.pack(t.y);
        return o;
    }
};

template <>
struct convert<QUuid> {
    msgpack::object const& operator()(msgpack::object const &o, QUuid &t) const {
        if (o.type != msgpack::type::ARRAY) throw msgpack::type_error();
        if (o.via.array.size != 1) throw msgpack::type_error();
        std::string str;
        o.via.array.ptr[0] >> str;
        t = QUuid::fromString(QString::fromStdString(str));
        return o;
    }
};

template <>
struct pack<QUuid> {
    template <typename Stream>
    msgpack::packer<Stream>& operator()(msgpack::packer<Stream> &o, QUuid const &t) const {
        o.pack_array(1);
        o.pack(t.toString().toStdString());
        return o;
    }
};

#if MSGPACK_VERSION_MAJOR <= 3 && MSGPACK_VERSION_MINOR < 2
template <>
struct as<std::chrono::system_clock::time_point> {
    typename std::chrono::system_clock::time_point operator()(msgpack::object const& o) const {
        if(o.type != msgpack::type::EXT) { throw msgpack::type_error(); }
        if(o.via.ext.type() != -1) { throw msgpack::type_error(); }
        std::chrono::system_clock::time_point tp;
        switch(o.via.ext.size) {
        case 4: {
            uint32_t sec;
            _msgpack_load32(uint32_t, o.via.ext.data(), &sec);
            tp += std::chrono::seconds(sec);
        } break;
        case 8: {
            uint64_t value;
            _msgpack_load64(uint64_t, o.via.ext.data(), &value);
            uint32_t nanosec = static_cast<uint32_t>(value >> 34);
            uint64_t sec = value & 0x00000003ffffffffLL;
            tp += std::chrono::duration_cast<std::chrono::system_clock::duration>(
                std::chrono::nanoseconds(nanosec));
            tp += std::chrono::seconds(sec);
        } break;
        case 12: {
            uint32_t nanosec;
            _msgpack_load32(uint32_t, o.via.ext.data(), &nanosec);
            int64_t sec;
            _msgpack_load64(int64_t, o.via.ext.data() + 4, &sec);
            tp += std::chrono::duration_cast<std::chrono::system_clock::duration>(
                std::chrono::nanoseconds(nanosec));
            tp += std::chrono::seconds(sec);
        } break;
        default:
            throw msgpack::type_error();
        }
        return tp;
    }
};

template <>
struct convert<std::chrono::system_clock::time_point> {
    msgpack::object const& operator()(msgpack::object const& o, std::chrono::system_clock::time_point& v) const {
        if(o.type != msgpack::type::EXT) { throw msgpack::type_error(); }
        if(o.via.ext.type() != -1) { throw msgpack::type_error(); }
        std::chrono::system_clock::time_point tp;
        switch(o.via.ext.size) {
        case 4: {
            uint32_t sec;
            _msgpack_load32(uint32_t, o.via.ext.data(), &sec);
            tp += std::chrono::seconds(sec);
            v = tp;
        } break;
        case 8: {
            uint64_t value;
            _msgpack_load64(uint64_t, o.via.ext.data(), &value);
            uint32_t nanosec = static_cast<uint32_t>(value >> 34);
            uint64_t sec = value & 0x00000003ffffffffLL;
            tp += std::chrono::duration_cast<std::chrono::system_clock::duration>(
                std::chrono::nanoseconds(nanosec));
            tp += std::chrono::seconds(sec);
            v = tp;
        } break;
        case 12: {
            uint32_t nanosec;
            _msgpack_load32(uint32_t, o.via.ext.data(), &nanosec);
            int64_t sec;
            _msgpack_load64(int64_t, o.via.ext.data() + 4, &sec);
            tp += std::chrono::duration_cast<std::chrono::system_clock::duration>(
                std::chrono::nanoseconds(nanosec));
            tp += std::chrono::seconds(sec);
            v = tp;
        } break;
        default:
            throw msgpack::type_error();
        }
        return o;
    }
};

template <>
struct pack<std::chrono::system_clock::time_point> {
    template <typename Stream>
    msgpack::packer<Stream>& operator()(msgpack::packer<Stream>& o, const std::chrono::system_clock::time_point& v) const {
        int64_t count = static_cast<int64_t>(v.time_since_epoch().count());

        int64_t nano_num =
            std::chrono::system_clock::duration::period::ratio::num *
            (1000000000 / std::chrono::system_clock::duration::period::ratio::den);

        int64_t nanosec = count % (1000000000 / nano_num) * nano_num;
        int64_t sec = 0;
        if (nanosec < 0) {
            nanosec = 1000000000 + nanosec;
            --sec;
        }
        sec += count
               * std::chrono::system_clock::duration::period::ratio::num
               / std::chrono::system_clock::duration::period::ratio::den;
        if ((sec >> 34) == 0) {
            uint64_t data64 = (static_cast<uint64_t>(nanosec) << 34) | static_cast<uint64_t>(sec);
            if ((data64 & 0xffffffff00000000L) == 0) {
                // timestamp 32
                o.pack_ext(4, -1);
                uint32_t data32 = static_cast<uint32_t>(data64);
                char buf[4];
                _msgpack_store32(buf, data32);
                o.pack_ext_body(buf, 4);
            }
            else {
                // timestamp 64
                o.pack_ext(8, -1);
                char buf[8];
                _msgpack_store64(buf, data64);
                o.pack_ext_body(buf, 8);
            }
        }
        else {
            // timestamp 96
            o.pack_ext(12, -1);
            char buf[12];
            _msgpack_store32(&buf[0], static_cast<uint32_t>(nanosec));
            _msgpack_store64(&buf[4], sec);
            o.pack_ext_body(buf, 12);
        }
        return o;
    }
};

template <>
struct object_with_zone<std::chrono::system_clock::time_point> {
    void operator()(msgpack::object::with_zone& o, const std::chrono::system_clock::time_point& v) const {
        int64_t count = static_cast<int64_t>(v.time_since_epoch().count());

        int64_t nano_num =
            std::chrono::system_clock::duration::period::ratio::num *
            (1000000000 / std::chrono::system_clock::duration::period::ratio::den);

        int64_t nanosec = count % (1000000000 / nano_num) * nano_num;
        int64_t sec = 0;
        if (nanosec < 0) {
            nanosec = 1000000000 + nanosec;
            --sec;
        }
        sec += count
               * std::chrono::system_clock::duration::period::ratio::num
               / std::chrono::system_clock::duration::period::ratio::den;
        if ((sec >> 34) == 0) {
            uint64_t data64 = (static_cast<uint64_t>(nanosec) << 34) | static_cast<uint64_t>(sec);
            if ((data64 & 0xffffffff00000000L) == 0) {
                // timestamp 32
                o.type = msgpack::type::EXT;
                o.via.ext.size = 4;
                char* p = static_cast<char*>(o.zone.allocate_no_align(o.via.ext.size + 1));
                p[0] = static_cast<char>(-1);
                uint32_t data32 = static_cast<uint32_t>(data64);
                _msgpack_store32(&p[1], data32);
                o.via.ext.ptr = p;
            }
            else {
                // timestamp 64
                o.type = msgpack::type::EXT;
                o.via.ext.size = 8;
                char* p = static_cast<char*>(o.zone.allocate_no_align(o.via.ext.size + 1));
                p[0] = static_cast<char>(-1);
                _msgpack_store64(&p[1], data64);
                o.via.ext.ptr = p;
            }
        }
        else {
            // timestamp 96
            o.type = msgpack::type::EXT;
            o.via.ext.size = 12;
            char* p = static_cast<char*>(o.zone.allocate_no_align(o.via.ext.size + 1));
            p[0] = static_cast<char>(-1);
            _msgpack_store32(&p[1], static_cast<uint32_t>(nanosec));
            _msgpack_store64(&p[1 + 4], sec);
            o.via.ext.ptr = p;
        }
    }
};
#endif

} // namespace adaptor
} // MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
} // namespace msgpack
