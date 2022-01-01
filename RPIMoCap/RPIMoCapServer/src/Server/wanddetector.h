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

#include <opencv2/core/mat.hpp>

#include <optional>
#include <vector>

class WandDetector
{
public:
    static std::optional<std::vector<cv::Point2f>> detect4pWand(const std::vector<cv::Point2f> &pts);
    static std::optional<std::vector<cv::Point2f>> detect3pWand(const std::vector<cv::Point2f> &pts);


    struct CrossDetection {
        size_t centerPointID;
        cv::Point2f centerPoint;
        std::vector<size_t> edgePointIDs;
        std::vector<cv::Point2f> edgePoints;
    };

    /**
     * @brief Detects center point on cross calibration wand
     * @param pts Required size - 5
     * @param epsilonDeg
     * @return CrossDetection with separated center point
     */
    static std::optional<CrossDetection> detectCrossCenter(const std::vector<cv::Point2f> &pts, const float epsilonDeg);


private:
    /**
     * @brief Given three collinear points p, q, r, the function checks if
     *        point q lies on line segment 'pr'
     * @param p
     * @param q
     * @param r
     * @return
     */
    static bool isOnSegment(cv::Point2f p, cv::Point2f q, cv::Point2f r);

    static bool areCollinear(cv::Point2f pnt1, cv::Point2f pnt2, cv::Point2f pnt3, float epsilonDeg);

    static float angle2D(cv::Point2f v1, cv::Point2f v2);
};
