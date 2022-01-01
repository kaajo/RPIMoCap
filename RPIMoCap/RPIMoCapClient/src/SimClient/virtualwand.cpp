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

#include <RPIMoCap/SimClient/virtualwand.h>

namespace RPIMoCap::SimClient {

VirtualExtrinsicWand::VirtualExtrinsicWand(float sizecm, float middlePointOffsetcm)
    : m_leftPoint(-sizecm/2.0f, 0.0f, 0.0f)
    , m_middlePoint(middlePointOffsetcm, 0.0f, 0.0f)
    , m_rightPoint(sizecm/2.0f, 0.0f, 0.0f)
{
}

std::vector<SimMarker> VirtualExtrinsicWand::markers(const Eigen::Affine3f &transform) const
{
    SimMarker left;
    const auto leftTr = transform * m_leftPoint;
    left.translation = cv::Point3f(leftTr.x(), leftTr.y(), leftTr.z());
    SimMarker middle;
    const auto middleTr = transform * m_middlePoint;
    middle.translation = cv::Point3f(middleTr.x(), middleTr.y(), middleTr.z());
    SimMarker right;
    const auto rightTr = transform * m_rightPoint;
    right.translation = cv::Point3f(rightTr.x(), rightTr.y(), rightTr.z());

    return {left, middle, right};
}

VirtualFloorWand::VirtualFloorWand(float sizecm)
    : m_centerPoint(0.0f, 0.0f, 0.0f)
    , m_leftPoint(sizecm, 0.0f, 0.0f)
    , m_rightPoint(-sizecm, 0.0f, 0.0f)
    , m_nearPoint(0.0f, 0.0f, -sizecm)
    , m_farPoint(0.0f, 0.0f, sizecm)
{

}

std::vector<SimMarker> VirtualFloorWand::markers(const Eigen::Affine3f &transform) const
{
    SimMarker center;
    const auto centerTr = transform * m_centerPoint;
    center.translation = cv::Point3f(centerTr.x(), centerTr.y(), centerTr.z());
    SimMarker left;
    const auto leftTr = transform * m_leftPoint;
    left.translation = cv::Point3f(leftTr.x(), leftTr.y(), leftTr.z());
    SimMarker right;
    const auto rightTr = transform * m_rightPoint;
    right.translation = cv::Point3f(rightTr.x(), rightTr.y(), rightTr.z());
    SimMarker near;
    const auto nearTr = transform * m_nearPoint;
    near.translation = cv::Point3f(nearTr.x(), nearTr.y(), nearTr.z());
    SimMarker far;
    const auto farTr = transform * m_farPoint;
    far.translation = cv::Point3f(farTr.x(), farTr.y(), farTr.z());

    return {center, left, right, near, far};
}

}
