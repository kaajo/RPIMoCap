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

namespace RPIMoCap {

bool closestPoints(const Line3D &line1, const Line3D &line2, Eigen::Vector3f &pointLine1, Eigen::Vector3f &pointLine2)
{
    const float a = line1.direction().dot(line1.direction());
    const float b = line1.direction().dot(line2.direction());
    const float e = line2.direction().dot(line2.direction());

    const float d = a*e - b*b;

    // if lines are not parallel
    if(std::abs(d) > 0.0000001f)
    {
        const Eigen::Vector3f r = line1.origin() - line2.origin();
        const float c = line1.direction().dot(r);
        const float f = line2.direction().dot(r);

        const float coeff1 = (b*f - c*e) / d;
        const float coeff2 = (a*f - c*b) / d;

        // if some of the point lies "behind" origin
        if (coeff1 < 0.0f || coeff2 < 0.0f)
        {
            return false;
        }

        pointLine1 = line1.origin() + line1.direction() * coeff1;
        pointLine2 = line2.origin() + line2.direction() * coeff2;

        return true;
    }
    else
    {
        return false;
    }
}

float lineAngle(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2)
{
    return std::atan2( v1.cross(v2).norm(), v1.dot(v2));
}

}
