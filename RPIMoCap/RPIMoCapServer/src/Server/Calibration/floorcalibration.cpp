/*
 * This file is part of the RPIMoCap (https://github.com/kaajo/RPIMoCap).
 * Copyright (c) 2021 Miroslav Krajicek.
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

#include "floorcalibration.h"

#include <Eigen/Eigenvalues>

namespace RPIMoCap {

/**
     * @brief converts between quaternion representation to euler angels.
     *
     * @param q Quaternion
     * @return Vector3[roll, pitch, yaw] in rad
     *
     * @note Pitch is only limited to +/-90° of freedom.
     *
     * source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_Angles_Conversion
     */
static Eigen::Vector3f toEulerAngles(const Eigen::Quaternionf& q) {
    // pitch (X-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    double pitch = std::atan2(sinr_cosp, cosr_cosp);

    // yaw (Y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    double yaw = std::abs(sinp) >= 1 ? std::copysign(M_PI / 2, sinp) : std::asin(sinp); // use 90 degrees if out of range

    // roll (Z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    double roll = std::atan2(siny_cosp, cosy_cosp);

    return {pitch, yaw, roll};
}

std::optional<Eigen::Affine3f> RPIMoCap::FloorCalibration::computeOrigin(std::vector<CameraData> data, float size) {

    std::vector<std::pair<QUuid, WandDetector::CrossDetection>> detections;

    float epsDeg = 5.0; // TODO add to parameters

    // Detect cross in all cameras
    for (auto &camData : data) {
        auto det = WandDetector::detectCrossCenter(camData.detectedPixels, epsDeg);

        if (det.has_value()) {
            detections.push_back({camData.id, det.value()});
        }
    }

    if (detections.size() < 2) {
        return std::nullopt;
    }

    // Find intersections for center point in every camera
    //TODO for more than two cameras
    auto cam1 = std::find_if(data.begin(), data.end(), [&](auto data){return data.id == detections[0].first;});
    auto cam2 = std::find_if(data.begin(), data.end(), [&](auto data){return data.id == detections[1].first;});

    Eigen::Vector3f pointLine1 = Eigen::Vector3f::Zero();
    Eigen::Vector3f pointLine2 = Eigen::Vector3f::Zero();

    bool intersection = closestPoints(cam1->rays[detections[0].second.centerPointID], cam2->rays[detections[1].second.centerPointID], pointLine1, pointLine2);

    if (!intersection) {
        return std::nullopt;
    }

    Eigen::Vector3f centerPoint((pointLine2 + pointLine1)/2);

    // Find correct combination of points
    std::vector<Line3D> baseRays = createRaysFromIDs(*cam1, detections[0].second.edgePointIDs);

    float minError = std::numeric_limits<float>::max();
    std::vector<size_t> bestIDs;
    std::vector<Line3D> bestRays;
    // check every rotation of rays
    std::vector<size_t> edgeIDs = detections[1].second.edgePointIDs;
    std::sort(edgeIDs.begin(), edgeIDs.end());
    do {
        const std::vector<Line3D> rays = createRaysFromIDs(*cam2, edgeIDs);
        auto distance = averageDistance(baseRays, rays, centerPoint, 2.0); // TODO threshold to params

        if (distance.has_value() && std::abs(distance.value() - size) < minError) {
            minError = std::abs(distance.value() - size);
            bestIDs = edgeIDs;
            bestRays = rays;
        }
    } while (std::next_permutation(edgeIDs.begin(), edgeIDs.end()));

    // make triangulation with best rotation
    assert(baseRays.size() == bestRays.size());
    std::vector<Eigen::Vector3f> points;
    for (size_t i = 0; i < baseRays.size(); ++i) {
        Eigen::Vector3f iPoint = Eigen::Vector3f::Zero();
        Eigen::Vector3f jPoint = Eigen::Vector3f::Zero();

        if (closestPoints(baseRays[i], bestRays[i], iPoint, jPoint)) {
            points.push_back((jPoint + iPoint)/2);
        } else {
            qWarning() << "No intersection for best candidate rays";
        }
    }

    // Compute normal vector to surface (Y axis) and use it for roll and pitch
    Eigen::Vector3f yAxis = computeNormalVector(points);

    Eigen::Quaternionf xz = Eigen::Quaternionf::Identity();
    xz.setFromTwoVectors(yAxis, Eigen::Vector3f{0.0f, 1.0f, 0.0f}); // TODO upvector to settings

    Eigen::Vector3f xzAngle = xz.toRotationMatrix().eulerAngles(0,1,2);

    //Find 2 other axis
    auto xzAxes = findXZAxes(points);

    if (!xzAxes.has_value()) {
        return std::nullopt;
    }

    // Get yaw
    Eigen::Vector3f zAxis = (Eigen::AngleAxisf(xzAngle[0], Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(xzAngle[2], Eigen::Vector3f::UnitZ()))
                            * Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    auto newZAxis = std::min_element(std::begin(xzAxes.value()), std::end(xzAxes.value()),
                                     [zAxis](const auto& left, const auto& right)
                                     { return lineAngle(left.direction(), zAxis) < lineAngle(right.direction(), zAxis);});

    Eigen::Quaternionf rot = Eigen::Quaternionf::FromTwoVectors(zAxis, newZAxis->direction());
    float yAngle = toEulerAngles(rot).y();

    // Final transform of
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(xzAngle[0], Eigen::Vector3f::UnitX())
                     * Eigen::AngleAxisf(yAngle, Eigen::Vector3f::UnitY())
                     * Eigen::AngleAxisf(xzAngle[2], Eigen::Vector3f::UnitZ()));

    transform.translation() = centerPoint;

    return transform;
}

std::optional<float> FloorCalibration::averageDistance(std::vector<Line3D> rays1, std::vector<Line3D> rays2,
                                                       const Eigen::Vector3f& centerPoint, const float distanceThreshold)
{
    if (rays1.size() != rays2.size() || rays1.empty()) {
        return std::nullopt;
    }

    std::vector<Eigen::Vector3f> points;

    for (size_t i = 0; i < rays1.size(); ++i) {
        Eigen::Vector3f iPoint = Eigen::Vector3f::Zero();
        Eigen::Vector3f jPoint = Eigen::Vector3f::Zero();

        if (closestPoints(rays1[i], rays2[i], iPoint, jPoint)) {

            if ((jPoint - iPoint).norm() > distanceThreshold) {
                return std::nullopt;
            }

            points.push_back((jPoint + iPoint)/2);
        } else {
            return std::nullopt;
        }
    }

    std::vector<float> distances;
    for (size_t i = 0; i < points.size(); ++i) {
        distances.push_back((centerPoint - points[i]).norm());
    }

    std::sort(distances.begin(), distances.end());

    float sum = 0.0f;
    for (size_t i = 0; i < points.size(); ++i) {
        sum += distances[i];
    }

    return sum / points.size();
}

std::vector<Line3D> FloorCalibration::createRaysFromIDs(const CameraData& camData, const std::vector<size_t> &ids)
{
    std::vector<Line3D> rays;

    for (size_t i = 0; i < ids.size(); ++i) {
        rays.push_back(camData.rays[ids[i]]);
    }

    return rays;
}

Eigen::Vector3f FloorCalibration::computeNormalVector(const std::vector<Eigen::Vector3f> &points)
{
    Eigen::Map<const Eigen::Matrix3Xf> P(&points[0].x(), 3, points.size());

    Eigen::Vector3f centroid = P.rowwise().mean();
    Eigen::MatrixXf centered = P.colwise() - centroid;
    Eigen::Matrix3f cov = centered * centered.transpose();

    //eigvecs sorted in increasing order of eigvals
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig;
    eig.computeDirect(cov);
    Eigen::Vector3f normal = eig.eigenvectors().col(0); //is already normalized
    if (normal(1) < 0.0f) {
        normal = -normal; //flip Y towards camera
    }

    return normal;
}

std::optional<std::array<Line3D, 4>> FloorCalibration::findXZAxes(const std::vector<Eigen::Vector3f>& crossPoints)
{
    if (crossPoints.size() != 4) {
        return std::nullopt;
    }

    // Find first axis
    const Eigen::Vector3f axis1point1 = crossPoints[0];
    auto axis1point2it = std::max_element(crossPoints.begin(), crossPoints.end(),
                                          [axis1point1](const auto& first, const auto& second)
                                          {return (axis1point1 - first).norm() < (axis1point1 - second).norm();});

    assert(axis1point2it != crossPoints.end());

    const size_t axis1point2idx = std::distance(crossPoints.begin(), axis1point2it);
    const Line3D firstAxis(axis1point1, crossPoints[axis1point2idx] - axis1point1);
    const Line3D firstAxisAlt(axis1point1, axis1point1 - crossPoints[axis1point2idx]);

    // Missing indexes -> 2nd axis
    std::set<size_t> secondAxisIndexes{1,2,3};
    secondAxisIndexes.erase(axis1point2idx);

    assert(secondAxisIndexes.size() == 2);

    const size_t axis2point1idx = *secondAxisIndexes.begin();
    const size_t axis2point2idx = *secondAxisIndexes.rbegin();

    const Line3D secondAxis(crossPoints[axis2point1idx], crossPoints[axis2point2idx] - crossPoints[axis2point1idx]);
    const Line3D secondAxisAlt(crossPoints[axis2point1idx], crossPoints[axis2point1idx] - crossPoints[axis2point2idx]);

    return std::array<Line3D, 4>{firstAxis, secondAxis, firstAxisAlt, secondAxisAlt};
}

}
