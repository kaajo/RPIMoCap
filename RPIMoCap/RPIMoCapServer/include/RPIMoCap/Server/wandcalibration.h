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

#include <RPIMoCap/Server/camerasettings.h>
#include <RPIMoCap/Server/wanddetector.h>
#include <RPIMoCap/Core/cameraparams.h>
#include <RPIMoCap/Core/msgpack_defs.h>

#include <QObject>
#include <QMap>

#include <opencv2/core/mat.hpp>
#include <ceres/ceres.h>

//TODO remove
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <ceres/rotation.h>
#include <iostream>

namespace RPIMoCap {

class ReprojectionError {
public:
    static ceres::CostFunction* Create(cv::Point2f observation, cv::Mat cameraMatrix) {
        return (new ceres::NumericDiffCostFunction<ReprojectionError, ceres::CENTRAL, 2, 6, 3>(
            new ReprojectionError(observation, cameraMatrix)));
    }

    bool operator()(const double* const camera, const double* const point, double* residuals) const
    {
        const cv::Vec3f rVec(camera[0], camera[1], camera[2]);
        const cv::Vec3f tVec(camera[3], camera[4], camera[5]);

        const cv::Affine3d cameraTransform(rVec, tVec);

        std::vector<cv::Vec3f> triangulatedPoint(1, cameraTransform * cv::Point3f(point[0], point[1], point[2]));

        std::vector<cv::Point2f> projPixels;
        cv::projectPoints(triangulatedPoint, cv::Vec3f::zeros(), cv::Vec3f::zeros(),
                          m_cameraMatrix, cv::noArray(), projPixels);

        const cv::Point2f res = projPixels.front() - m_observation;

        if (triangulatedPoint[0][2] > 0.0f)
        {
            residuals[0] = res.x;
            residuals[1] = res.y;
        }
        else
        {
            residuals[0] = 1000000.0 - res.x;
            residuals[1] = 1000000.0 - res.y;
        }

        return true;
    }

private:
    ReprojectionError(cv::Point2f observation, cv::Mat cameraMatrix)
        : m_observation(observation)
        , m_cameraMatrix(cameraMatrix) {}

    cv::Point2f m_observation;
    cv::Mat m_cameraMatrix;
};


class PointDistanceError {
public:
    static ceres::CostFunction* Create(float middleToLeftDist, float middleToRightDist, float leftToRightDist) {
        return (new ceres::AutoDiffCostFunction<PointDistanceError, 3, 3, 3, 3>(
            new PointDistanceError(middleToLeftDist, middleToRightDist, leftToRightDist)));
    }

    template <typename T>
    bool operator()(const T* const pointLeft, const T* const pointMiddle, const T* const pointRight, T* residuals) const
    {
        T leftMiddleX = pointMiddle[0] - pointLeft[0];
        T leftMiddleY = pointMiddle[1] - pointLeft[1];
        T leftMiddleZ = pointMiddle[2] - pointLeft[2];

        T rightMiddleX = pointMiddle[0] - pointRight[0];
        T rightMiddleY = pointMiddle[1] - pointRight[1];
        T rightMiddleZ = pointMiddle[2] - pointRight[2];

        T leftRightX = pointLeft[0] - pointRight[0];
        T leftRightY = pointLeft[1] - pointRight[1];
        T leftRightZ = pointLeft[2] - pointRight[2];

        residuals[0] = T(m_middleToLeftDistSq) - (leftMiddleX*leftMiddleX + leftMiddleY*leftMiddleY + leftMiddleZ*leftMiddleZ);
        residuals[1] = T(m_middleToRightDistSq) - (rightMiddleX*rightMiddleX + rightMiddleY*rightMiddleY + rightMiddleZ*rightMiddleZ);
        residuals[2] = T(m_leftToRightDistSq) - (leftRightX*leftRightX + leftRightY*leftRightY + leftRightZ*leftRightZ);

        return true;
    }

private:
    PointDistanceError(float middleToLeftDist, float middleToRightDist, float leftToRightDist)
        : m_middleToLeftDistSq(middleToLeftDist * middleToLeftDist)
        , m_middleToRightDistSq(middleToRightDist * middleToRightDist)
        , m_leftToRightDistSq(leftToRightDist * leftToRightDist) {}

    float m_middleToLeftDistSq;
    float m_middleToRightDistSq;
    float m_leftToRightDistSq;
};

struct ObsDetection {
    cv::Mat cameraMatrix;

    std::vector<cv::Point2f> firstPixels;
    std::vector<cv::Point2f> secondPixels;
    std::vector<cv::Point3d> triangulatedPoints;

    Eigen::Vector3f firstRVec = Eigen::Vector3f::Zero();
    Eigen::Vector3f secondRVec = Eigen::Vector3f::Zero();

    Eigen::Vector3f firstTVec = Eigen::Vector3f::Zero();
    Eigen::Vector3f secondTVec = Eigen::Vector3f::Zero();

    float reprojectionError()
    {
        return (firstReprojectionError() + secondReprojectionError())/2.0f;
    }

    float firstReprojectionError()
    {
        return reprojectionError(firstPixels, triangulatedPoints,
                                 cv::Vec3f(firstRVec.x(), firstRVec.y(), firstRVec.z()),
                                 cv::Vec3f(firstTVec.x(), firstTVec.y(), firstTVec.z()),
                                 cameraMatrix);
    }

    float secondReprojectionError()
    {
        return reprojectionError(secondPixels, triangulatedPoints,
                                 cv::Vec3f(secondRVec.x(), secondRVec.y(), secondRVec.z()),
                                 cv::Vec3f(secondTVec.x(), secondTVec.y(), secondTVec.z()),
                                 cameraMatrix);
    }

    cv::Mat firstProjMat() {
        cv::Vec3d rVec(firstRVec.x(), firstRVec.y(), firstRVec.z());
        cv::Vec3d tVec(firstTVec.x(), firstTVec.y(), firstTVec.z());
        auto mat = cv::Mat(cv::Affine3d(rVec, tVec).matrix);
        return cameraMatrix * mat(cv::Rect(0,0,4,3));
    }

    cv::Mat secondProjMat() {
        cv::Vec3d rVec(secondRVec.x(), secondRVec.y(), secondRVec.z());
        cv::Vec3d tVec(secondTVec.x(), secondTVec.y(), secondTVec.z());
        auto mat = cv::Mat(cv::Affine3d(rVec, tVec).matrix);
        return cameraMatrix * mat(cv::Rect(0,0,4,3));
    }

    Eigen::Affine3f relativeTransform() const
    {
        auto rVec = secondRVec - firstRVec;
        auto tVec = secondTVec - firstTVec;

        Eigen::Matrix3f rot;
        rot = Eigen::AngleAxisf(rVec[0], Eigen::Vector3f::UnitX())
              * Eigen::AngleAxisf(rVec[1], Eigen::Vector3f::UnitY())
              * Eigen::AngleAxisf(rVec[2], Eigen::Vector3f::UnitZ());

        Eigen::Vector3f translation(tVec[0], tVec[1], tVec[2]);

        Eigen::Affine3f t = Eigen::Affine3f::Identity();
        t.fromPositionOrientationScale(translation, rot, Eigen::Vector3f(1.0, 1.0, 1.0));

        return t;
    }

    double* cameraFirstParams(double focalLength)
    {
        cameraFirstData[0] = firstRVec[0];
        cameraFirstData[1] = firstRVec[1];
        cameraFirstData[2] = firstRVec[2];
        cameraFirstData[3] = firstTVec[0];
        cameraFirstData[4] = firstTVec[1];
        cameraFirstData[5] = firstTVec[2];
        return &cameraFirstData[0];
    }

    double* cameraSecondParams(double focalLength)
    {
        cameraSecondData[0] = secondRVec[0];
        cameraSecondData[1] = secondRVec[1];
        cameraSecondData[2] = secondRVec[2];
        cameraSecondData[3] = secondTVec[0];
        cameraSecondData[4] = secondTVec[1];
        cameraSecondData[5] = secondTVec[2];
        return &cameraSecondData[0];
    }

private:
    static float reprojectionError(const std::vector<cv::Point2f> &pixels,
                            const std::vector<cv::Point3d> &triangulatedPoints,
                            const cv::Vec3f &rVec, const cv::Vec3f &tVec,
                            const cv::Mat &cameraMatrix)
    {
        std::vector<cv::Point3f> pts;
        std::transform(triangulatedPoints.begin(), triangulatedPoints.end(), std::back_inserter(pts),
                       [](auto &pnt){return cv::Point3d(pnt);});

        if (pts.empty())
        {
            return std::numeric_limits<float>::infinity();
        }

        std::vector<cv::Point2f> projPixels;
        cv::projectPoints(pts, rVec, tVec,
                          cameraMatrix, cv::noArray(), projPixels);
        assert(projPixels.size() == pixels.size());

        if (pixels.empty())
        {
            return std::numeric_limits<float>::infinity();
        }

        float sum = 0.0f;
        for (size_t i = 0; i < pixels.size(); ++i)
        {
            sum += cv::norm(projPixels[i] - pixels[i]);
        }

        return sum/pixels.size();
    }

    std::array<double,9> cameraFirstData;
    std::array<double,9> cameraSecondData;
};

class WandCalibration : public QObject
{
    Q_OBJECT
public:
    WandCalibration(QMap<QUuid,std::shared_ptr<CameraSettings>> &cameraSettings,
                    RPIMoCap::Camera::Intrinsics camData, QObject *parent = nullptr);

    void addFrame(const std::vector<std::pair<QUuid, std::vector<cv::Point2f> > > &points);

    static QMap<QUuid, Eigen::Affine3f> relativeToGlobalTransforms(const QMap<std::pair<QUuid, QUuid>, ObsDetection> &detections);

private:
    float computeScale(std::vector<cv::Point3d> &triangulatedPoints);
    void triangulatePoints();

    std::vector<cv::Point3f> m_wandPoints;

    QMap<QUuid,std::shared_ptr<CameraSettings>> m_cameraSettings;

    bool finished = false;

    float m_errorTreshold = 3.0f; //reprojection error threshold for second stage

    bool isCameraPairReady(const ObsDetection &detections);
    bool isReadyForPreciseStage();
    size_t m_minCalibObservations = 300; //minimum number of observed frames for a pair of cameras

    void addObservations(const std::vector<std::pair<QUuid, std::vector<cv::Point2f>>> &wandPoints);
    cv::Mat computeBasicRelativeTransform(std::vector<cv::Point2f> firstPixels, std::vector<cv::Point2f> secondPixels, cv::Mat &cameraMatrix);

    QMap<std::pair<QUuid,QUuid>,ObsDetection> m_observedDetections;

    RPIMoCap::Camera::Intrinsics m_camData;
};

}
