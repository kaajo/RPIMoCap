#include "Server/wanddetector.h"

#include <opencv2/core.hpp>
#include <QSet>

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <iostream>

std::optional<std::vector<cv::Point2f> > WandDetector::detect4pWand(const std::vector<cv::Point2f> &pts)
{
    if (pts.size() != 4)
    {
        return std::nullopt;
    }

    cv::Mat distMap(pts.size(), pts.size(), CV_32FC1,
                    cv::Scalar(std::numeric_limits<float>::infinity()));

    for (size_t i = 0; i < pts.size(); ++i)
    {
        for (size_t j = i+1; j < pts.size(); ++j)
        {
            distMap.at<float>(i,j) = distMap.at<float>(j,i) = cv::norm(pts[i] - pts[j]);
        }
    }

    double min = std::numeric_limits<double>::lowest();
    double max = std::numeric_limits<double>::max();
    cv::Point minLoc(-1, -1);
    cv::Point maxLoc(-1, -1);
    cv::Mat diagMask = cv::Scalar::all(1) - cv::Mat::eye(pts.size(), pts.size(), CV_8UC1);
    cv::minMaxLoc(distMap, &min, &max, &minLoc, &maxLoc, diagMask);

    size_t borderL = maxLoc.x;
    size_t borderR = maxLoc.y;
    size_t middle = minLoc.x;
    size_t cross = minLoc.y;

    if (distMap.at<float>(cross,borderL) <  distMap.at<float>(middle,borderL))
    {
        std::swap(cross, middle);
    }

    if (distMap.at<float>(middle,borderL) <  distMap.at<float>(middle,borderR))
    {
        std::swap(borderL, borderR);
    }

    return std::vector<cv::Point2f>{pts[borderL], pts[middle], pts[borderR], pts[cross]};
}

std::optional<std::vector<cv::Point2f> > WandDetector::detect3pWand(const std::vector<cv::Point2f> &pts)
{
    if (pts.size() != 3)
    {
        return std::nullopt;
    }

    std::vector<float> distances;
    distances.push_back(cv::norm(pts[1] - pts[0]) + cv::norm(pts[2] - pts[0]));
    distances.push_back(cv::norm(pts[0] - pts[1]) + cv::norm(pts[2] - pts[1]));
    distances.push_back(cv::norm(pts[0] - pts[2]) + cv::norm(pts[1] - pts[2]));

    //TODO take also wand points for correct order
    const size_t middleID = (std::min_element(distances.begin(), distances.end()) - distances.begin());
    const size_t leftID = (std::max_element(distances.begin(), distances.end()) - distances.begin());

    QSet<size_t> indexes{0, 1, 2};
    indexes.remove(middleID);
    indexes.remove(leftID);
    const size_t rightID = indexes.values().first();

    return std::vector<cv::Point2f>{pts[leftID], pts[middleID], pts[rightID]};
}

std::optional<WandDetector::CrossDetection> WandDetector::detectCrossCenter(const std::vector<cv::Point2f> &pts, float epsilonDeg)
{
    if (pts.size() != 5)
    {
        return std::nullopt;
    }

    std::vector<std::tuple<size_t, size_t, size_t>> lines;

    // test colinearity of points (3 points)
    for (size_t firstID = 0; firstID < 5; ++firstID) {
        for (size_t secondID = firstID + 1; secondID  < 5; ++secondID ) {
            for (size_t thirdID = secondID + 1; thirdID < 5; ++thirdID) {

                const cv::Point2f& first = pts[firstID];
                const cv::Point2f& second = pts[secondID];
                const cv::Point2f& third = pts[thirdID];

                const bool collinear = areCollinear(first, second, third, epsilonDeg);
                // TODO rewrite algorithm so that areCollinear function returns epsilon and this function
                // takes 2 lowest values and compare it with epsilonDeg
                if (!collinear) {
                    continue;
                }

                // check if some of the 3 points is middle point on segment
                if (isOnSegment(first, second, third)) {
                    lines.push_back({firstID, secondID, thirdID});
                } else if (isOnSegment(second, first, third)) {
                    lines.push_back({secondID, firstID, thirdID});
                } else if (isOnSegment(second, third, first)) {
                    lines.push_back({secondID, thirdID, firstID});
                } else {
                    //TODO
                    std::cout << "something went wrong in cross detection" << std::endl;
                }
            }
        }
    }


    // expected 2 point triplets
    if (lines.size() != 2) {
        return std::nullopt;
    }

    // second point should be the same in both
    if (std::get<1>(lines[0]) != std::get<1>(lines[1])) {
        return std::nullopt;
    }

    // cross is symetric, return random order
    WandDetector::CrossDetection detection;
    detection.centerPoint = pts[std::get<1>(lines[0])];
    detection.centerPointID = std::get<1>(lines[0]);

    detection.edgePoints.push_back(pts[std::get<0>(lines[0])]);
    detection.edgePointIDs.push_back(std::get<0>(lines[0]));

    detection.edgePoints.push_back(pts[std::get<2>(lines[0])]);
    detection.edgePointIDs.push_back(std::get<2>(lines[0]));

    detection.edgePoints.push_back(pts[std::get<0>(lines[1])]);
    detection.edgePointIDs.push_back(std::get<0>(lines[1]));

    detection.edgePoints.push_back(pts[std::get<2>(lines[1])]);
    detection.edgePointIDs.push_back(std::get<2>(lines[1]));

    return detection;
}

bool WandDetector::isOnSegment(cv::Point2f p, cv::Point2f q, cv::Point2f r)
{
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
        q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y)) {
        return true;
    }

    return false;
}

bool WandDetector::areCollinear(cv::Point2f pnt1, cv::Point2f pnt2, cv::Point2f pnt3, float epsilonDeg)
{
    auto vec12(pnt2 - pnt1);
    auto vec21(pnt1 - pnt2);
    auto vec13(pnt3 - pnt1);
    auto vec31(pnt1 - pnt3);

    auto firstDir = std::abs(angle2D(vec12, vec13));
    auto secondDir = std::abs(angle2D(vec21, vec13));
    auto thirdDir = std::abs(angle2D(vec12, vec31)); // TODO needed?
    auto fourthDir = std::abs(angle2D(vec21, vec31)); // TODO needed?

    float minAngle = std::min({firstDir, secondDir, thirdDir, fourthDir});
    float epsRad = epsilonDeg / 180 * M_PI;

    return minAngle < epsRad;
}

float WandDetector::angle2D(cv::Point2f v1, cv::Point2f v2) {
    float angle = atan2(v2.y, v2.x) - atan2(v1.y, v1.x);

//    if (angle < 0.0f) {
//        angle += 2 * M_PI;
//    }

    return angle;
}
