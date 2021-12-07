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

#include <RPIMoCap/Core/mqttsubscriber.h>
#include <RPIMoCap/Core/line3d.h>
#include <RPIMoCap/Core/msgpack_defs.h>
#include <RPIMoCap/Core/cameraparams.h>
#include <RPIMoCap/Core/topics.h>

#include <QObject>
#include <QUuid>

#include <Eigen/Geometry>
#include <msgpack.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/affine.hpp>

#include <memory>
#include <mutex>

Q_DECLARE_METATYPE(Eigen::Vector3d)

namespace RPIMoCap {

struct CameraSettings : public QObject
{
    Q_OBJECT

public:
    Q_PROPERTY(Eigen::Vector3d translation MEMBER m_translation NOTIFY translationChanged)
    Q_PROPERTY(Eigen::Vector3d rotation MEMBER m_rotation NOTIFY rotationChanged)

    CameraSettings(QUuid id, const RPIMoCap::MQTTSettings &settings, Camera::Intrinsics camParams);

    virtual ~CameraSettings() = default;

    QUuid id() const {return m_id;}

    Eigen::Affine3f transform() const;

    Line3D computePixelRay(cv::Point2f pixel);

    Eigen::Matrix<double, 3, 3> essentialMatrix(const CameraSettings& targetCamera);

    cv::Point2f normalizeCoords(cv::Point2f pixel);

    Eigen::ParametrizedLine<double, 2> epipolarLine(cv::Point2f pixelNormalized, const Eigen::Matrix<double, 3, 3>& essentialMatrix);

signals:
    void raysReceived(const QUuid cameraID, const std::vector<std::pair<cv::Point2f, Line3D>> &rays);
    void translationChanged(const Eigen::Vector3d &tVec);
    void rotationChanged(const Eigen::Vector3d &rVec);

public slots:
    void setTranslation(Eigen::Vector3d tVec);

    void setRotation(Eigen::Vector3d rVec);

private slots:
    void onPointsDataReceived(const QByteArray &data);

private:
    Eigen::Matrix<double, 3, 3> cameraMatFromCV(const cv::Mat camMatrix);

    QUuid m_id;
    MQTTSubscriber m_pointSub;
    Camera::Intrinsics m_params;

    Eigen::Vector3d m_translation = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_rotation = Eigen::Vector3d::Zero();
};

}
