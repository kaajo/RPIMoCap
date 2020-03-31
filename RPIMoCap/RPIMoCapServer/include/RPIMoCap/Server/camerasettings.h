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

#include <QObject>
#include <QUuid>

#include <Eigen/Geometry>
#include <msgpack.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/affine.hpp>

#include <memory>
#include <mutex>

Q_DECLARE_METATYPE(cv::Vec3f)

namespace RPIMoCap {

struct CameraSettings : public QObject
{
    Q_OBJECT
public:
    CameraSettings(QUuid id, const RPIMoCap::MQTTSettings &settings, Camera::Intrinsics camParams)
        : m_id(id)
        , m_pointSub("serverPointsSub-" + id.toString(QUuid::StringFormat::WithoutBraces),
                     "/client-" + id.toString(QUuid::StringFormat::WithoutBraces) + "/points",settings)
        , m_params(std::move(camParams))
    {
        connect(&m_pointSub,&RPIMoCap::MQTTSubscriber::messageReceived, this, &CameraSettings::onPointsDataReceived);
    }

    virtual ~CameraSettings() = default;

    QUuid id() const {return m_id;}

    Eigen::Affine3f transform() const
    {
        Eigen::Affine3f t = Eigen::Affine3f::Identity();

        Eigen::Matrix3f rot;
        rot = Eigen::AngleAxisf(m_rotation[0], Eigen::Vector3f::UnitX())
              * Eigen::AngleAxisf(m_rotation[1], Eigen::Vector3f::UnitY())
              * Eigen::AngleAxisf(m_rotation[2], Eigen::Vector3f::UnitZ());
        t.rotate(rot);

        t.translation().x() = m_translation[0];
        t.translation().y() = m_translation[1];
        t.translation().z() = m_translation[2];

        return t;
    }

    Q_PROPERTY(cv::Vec3f translation MEMBER m_translation NOTIFY translationChanged)
    Q_PROPERTY(cv::Vec3f rotation MEMBER m_rotation NOTIFY rotationChanged)

signals:
    void raysReceived(const QUuid cameraID, const std::vector<Line3D> &rays);
    void translationChanged(const cv::Vec3f &tVec);
    void rotationChanged(const cv::Vec3f &rVec);

private slots:
    void onPointsDataReceived(const QByteArray &data)
    {
        msgpack::object_handle result;
        msgpack::unpack(result, data.data(), data.length());

        const std::vector<cv::Point2f> points(result.get().as<std::vector<cv::Point2f>>());

        std::vector<Line3D> rays;
        for (auto &pnt : points)
        {
            rays.push_back(computePixelRay(pnt));
        }

        emit raysReceived(m_id, rays);
    }

private:
    Line3D computePixelRay(cv::Point2f pixel)
    {
        cv::Mat normalized = m_params.cameraMatrixInv * cv::Mat(cv::Vec3f(pixel.x, pixel.y, 1));

        Eigen::Vector3f origin(m_translation[0], m_translation[1], m_translation[2]);
        auto dir = cv::Affine3f(m_rotation) * cv::Vec3f(normalized.at<float>(0), normalized.at<float>(1), normalized.at<float>(2));

        return Line3D(origin, Eigen::Vector3f(dir[0], dir[1], dir[2]).normalized()); //TODO more effective!!!
    }

    QUuid m_id;
    MQTTSubscriber m_pointSub;
    Camera::Intrinsics m_params;

    cv::Vec3f m_translation = cv::Vec3f(0.0, 0.0, 0.0);
    cv::Vec3f m_rotation = cv::Vec3f(0.0, 0.0, 0.0);
};

}
