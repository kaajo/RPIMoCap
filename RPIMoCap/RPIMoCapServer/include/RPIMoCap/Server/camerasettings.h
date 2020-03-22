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

#include <QObject>
#include <QUuid>

#include <Eigen/Geometry>
#include <msgpack.hpp>
#include <opencv2/core/mat.hpp>

#include <memory>
#include <mutex>

struct CameraSettings : public QObject
{
    Q_OBJECT
public:
    CameraSettings(QUuid id, const RPIMoCap::MQTTSettings &settings, Eigen::Affine3f transform = Eigen::Affine3f::Identity())
        : m_id(id)
        , m_pointSub("serverPointsSub-" + id.toString(QUuid::StringFormat::WithoutBraces),
                     "/client-" + id.toString(QUuid::StringFormat::WithoutBraces) + "/points",settings)
        , m_transform(transform)
    {
        connect(&m_pointSub,&RPIMoCap::MQTTSubscriber::messageReceived, this, &CameraSettings::onPointsDataReceived);
    }

    virtual ~CameraSettings() {}

    QUuid id() const {return m_id;}

    Eigen::Affine3f transform() const
    {
        std::unique_lock lock(m_transformMutex);
        return m_transform;
    }

    void setTransform(const Eigen::Affine3f &newTransform)
    {
        {
            std::unique_lock lock(m_transformMutex);
            m_transform = newTransform;
        }
        emit changed();
    }

signals:
    void pointsReceived(const QUuid cameraID, const std::vector<cv::Point2f> &points);
    void changed();

private slots:
    void onPointsDataReceived(const QByteArray &data)
    {
        msgpack::object_handle result;
        msgpack::unpack(result, data.data(), data.length());

        const std::vector<cv::Point2f> points(result.get().as<std::vector<cv::Point2f>>());
        emit pointsReceived(m_id, points);
    }

private:
    mutable std::mutex m_transformMutex;

    QUuid m_id;
    RPIMoCap::MQTTSubscriber m_pointSub;
    Eigen::Affine3f m_transform;
};
