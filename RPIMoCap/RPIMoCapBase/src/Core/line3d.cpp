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

#include "RPIMoCap/Core/line3d.h"

bool RPIMoCap::closestPointsTwoLines(const RPIMoCap::Line3D &line1, const RPIMoCap::Line3D &line2, Eigen::Vector3f &closestPointLine1, Eigen::Vector3f &closestPointLine2)
{
    double a = line1.direction().dot(line1.direction());
    double b = line1.direction().dot(line2.direction());
    double e = line2.direction().dot(line2.direction());

    double d = a*e - b*b;

    // if lines are not parallel
    if(d != 0)
    {
        const Eigen::Vector3f r = line1.origin() - line2.origin();
        double c = line1.direction().dot(r);
        double f = line2.direction().dot(r);

        closestPointLine1 = line1.origin() + line1.direction() * (b*f - c*e) / d;
        closestPointLine2 = line2.origin() + line2.direction() * (a*f - c*b) / d;

        return true;
    }

    return false;
}

Eigen::Vector3f RPIMoCap::averagePoint(const Eigen::Vector3f &point1, const Eigen::Vector3f &point2)
{
    return Eigen::Vector3f((point1.x() + point2.x())/2,(point1.y() + point2.y())/2,(point1.z() + point2.z())/2);
}

float RPIMoCap::lineAngle(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2)
{
    return std::atan2( v1.cross(v2).norm(), v1.dot(v2));
}

float RPIMoCap::lineAngle(const Eigen::Vector2f &v1, const Eigen::Vector2f &v2)
{
    return atan2(v2.y(), v2.x()) - atan2(v1.y(), v1.x());
}

bool RPIMoCap::isIntersection(const RPIMoCap::Line3D &l1, const RPIMoCap::Line3D &l2, const float maxError, Eigen::Vector3f &point)
{
    Eigen::Vector3f point1, point2;

    if(! closestPointsTwoLines(l1, l2, point1, point2))
    {
        return false;
    }

    if(maxError > (point1 - point2).norm())
    {
        //l1.m_numberOfIntersections += 1;
        //l2.m_numberOfIntersections += 1;

        point = averagePoint(point1, point2);

        return true;
    }

    return false;
}
