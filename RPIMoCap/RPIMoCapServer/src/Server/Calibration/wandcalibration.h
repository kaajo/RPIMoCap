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

#include <Server/camerasettings.h>
#include <Server/wanddetector.h>
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

class Observation {
public:
    std::vector<cv::Point2f> pixels;
    Eigen::Vector3f rVec = Eigen::Vector3f::Zero();
    Eigen::Vector3f tVec = Eigen::Vector3f::Zero();

    cv::Mat projectionMatrix(const cv::Mat &cameraMatrix) {
        const cv::Vec3d rotVec(rVec.x(), rVec.y(), rVec.z());
        const cv::Vec3d trVec(tVec.x(), tVec.y(), tVec.z());
        auto mat = cv::Mat(cv::Affine3d(rotVec, trVec).matrix);
        return cameraMatrix * mat(cv::Rect(0,0,4,3));
    }

    float reprojectionError(const std::vector<cv::Point3d> &triangulatedPoints,
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
        cv::projectPoints(pts, cv::Vec3f(rVec.x(), rVec.y(), rVec.z()),
                          cv::Vec3f(tVec.x(), tVec.y(), tVec.z()),
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

    std::array<double, 6> ceresParams()
    {
        std::array<double, 6> retVal;
        retVal[0] = rVec[0];
        retVal[1] = rVec[1];
        retVal[2] = rVec[2];
        retVal[3] = tVec[0];
        retVal[4] = tVec[1];
        retVal[5] = tVec[2];
        return retVal;
    }

};

class ObservationPair : public std::pair<Observation, Observation> {
public:
    cv::Mat cameraMatrix;
    std::vector<cv::Point3d> triangulatedPoints;

    float reprojectionError()
    {
        return (first.reprojectionError(triangulatedPoints, cameraMatrix) +
                second.reprojectionError(triangulatedPoints, cameraMatrix))/2.0f;
    }

    Eigen::Affine3f relativeTransform() const
    {
        auto rVec = second.rVec - first.rVec;
        auto tVec = second.tVec - first.tVec;

        Eigen::Matrix3f rot;
        rot = Eigen::AngleAxisf(rVec[0], Eigen::Vector3f::UnitX())
              * Eigen::AngleAxisf(rVec[1], Eigen::Vector3f::UnitY())
              * Eigen::AngleAxisf(rVec[2], Eigen::Vector3f::UnitZ());

        Eigen::Vector3f translation(tVec[0], tVec[1], tVec[2]);

        Eigen::Affine3f t = Eigen::Affine3f::Identity();
        t.fromPositionOrientationScale(translation, rot, Eigen::Vector3f(1.0, 1.0, 1.0));
        return t;
    }
};

class WandCalibration : public QObject
{
    Q_OBJECT
public:
    enum class Type {
        Full,
        Refine
    };

    struct Settings {
        int framesPerCamera = -1;
        Type calibType = Type::Full;
    };

    struct InputData {
        std::vector<cv::Point3f> wandPoints;
        RPIMoCap::Camera::Intrinsics camParams;
        QMap<QUuid,std::shared_ptr<CameraSettings>> cameraSettings;
    };

    WandCalibration(QObject *parent = nullptr);

    bool running() const {return started;}

    void addFrame(const std::vector<std::pair<QUuid, std::vector<cv::Point2f> > > &points);

    static QMap<QUuid, Eigen::Affine3f> relativeToGlobalTransforms(const QMap<std::pair<QUuid, QUuid>, ObservationPair> &detections);

public slots:
    void startCalib(bool start, Settings settings, InputData data);

private:
    bool started = false;

    float computeScale(std::vector<cv::Point3d> &triangulatedPoints);
    void triangulatePoints();

    std::vector<cv::Point3f> m_wandPoints;

    QMap<QUuid,std::shared_ptr<CameraSettings>> m_cameraSettings;

    bool finished = false;

    float m_errorTreshold = 3.0f; //reprojection error threshold for second stage

    bool isCameraPairReady(const ObservationPair &detections);
    bool isReadyForPreciseStage();
    size_t m_minCalibObservations = 300; //minimum number of observed frames for a pair of cameras

    void addObservations(const std::vector<std::pair<QUuid, std::vector<cv::Point2f>>> &wandPoints);
    cv::Mat computeBasicRelativeTransform(std::vector<cv::Point2f> firstPixels, std::vector<cv::Point2f> secondPixels, cv::Mat &cameraMatrix);

    QMap<std::pair<QUuid,QUuid>,ObservationPair> m_observedDetections;

    RPIMoCap::Camera::Intrinsics m_camData;
};

}
