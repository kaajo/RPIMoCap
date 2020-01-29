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


class WandCalibration : public QObject
{
    Q_OBJECT
public:
    WandCalibration(QMap<int,std::shared_ptr<CameraSettings>> &cameraSettings,
                    RPIMoCap::CameraParams camData, QObject *parent = nullptr);

    void addFrame(const QMap<int, std::vector<cv::Point2f>> &points);

    static float computeReprojectionError(const std::vector<cv::Point2f> &pixels, const std::vector<cv::Point3d> &triangulatedPoints,
                                   const Eigen::Affine3f &estTransform, cv::Mat cameraMatrix);
private:
    float computeScale(const cv::Mat &transformEstimation, std::vector<cv::Point3d> &triangulatedPoints);

    std::vector<cv::Point3f> m_wandPoints;

    QMap<int,std::shared_ptr<CameraSettings>> m_cameraSettings;
    //QMap<int, CameraCalibState> m_extrinsicGuess;

    bool finished = false;

    float m_errorTreshold = 3.0f;

    struct ObsDetection {
        std::vector<cv::Point2f> firstPixels;
        std::vector<cv::Point2f> secondPixels;
        std::vector<cv::Point3d> triangulatedPoints;
        float reprojectionError = std::numeric_limits<float>::max();
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();

        std::array<double,9> cameraFirstData;
        double* cameraFirstParams(double focalLength)
        {
            cameraFirstData[0] = 0.0;
            cameraFirstData[1] = 0.0;
            cameraFirstData[2] = 0.0;
            cameraFirstData[3] = 0.0;
            cameraFirstData[4] = 0.0;
            cameraFirstData[5] = 0.0;

            return &cameraFirstData[0];
        }

        std::array<double,9> cameraSecondData;
        double* cameraSecondParams(double focalLength)
        {
            cv::Mat secondTransform;
            cv::eigen2cv(transform.matrix(), secondTransform);
            secondTransform.convertTo(secondTransform, CV_64FC1);

            cv::Mat rotVec;
            cv::Rodrigues(secondTransform.rowRange(0,3).colRange(0,3), rotVec);

            cameraSecondData[0] = rotVec.at<double>(0);
            cameraSecondData[1] = rotVec.at<double>(1);
            cameraSecondData[2] = rotVec.at<double>(2);
            cameraSecondData[3] = transform.translation().x();
            cameraSecondData[4] = transform.translation().y();
            cameraSecondData[5] = transform.translation().z();

            return &cameraSecondData[0];
        }

    };

    QMap<std::pair<int,int>,ObsDetection> m_observedDetections;

    RPIMoCap::CameraParams m_camData;
};
