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

#include <Eigen/Geometry>
#include <msgpack.hpp>
#include <opencv2/core/mat.hpp>

#include <memory>
#include <mutex>

struct CameraSettings : public QObject
{
    Q_OBJECT
public:
    CameraSettings(int id, const RPIMoCap::MQTTSettings &settings, Eigen::Affine3f transform = Eigen::Affine3f::Identity())
        : m_id(id)
        , m_lineSub("serverLinesSub" + QString::number(id), "/client" + QString::number(id) + "/lines",settings)
        , m_pointSub("serverPointsSub" + QString::number(id), "/client" + QString::number(id) + "/points",settings)
        , m_transform(transform)
    {
        connect(&m_lineSub,&RPIMoCap::MQTTSubscriber::messageReceived, this, &CameraSettings::onLinesDataReceived);
        connect(&m_pointSub,&RPIMoCap::MQTTSubscriber::messageReceived, this, &CameraSettings::onPointsDataReceived);
    }

    virtual ~CameraSettings() {}

    int id() const {return m_id;}

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
    void linesReceived(int cameraID, std::vector<RPIMoCap::Line3D> lines);
    void pointsReceived(int cameraID, const std::vector<cv::Point2f> &points);
    void changed();

private slots:
    void onLinesDataReceived(const QByteArray &data)
    {
        msgpack::object_handle result;
        msgpack::unpack(result, data.data(), data.length());

        std::vector<RPIMoCap::Line3D> lines(result.get().as<std::vector<RPIMoCap::Line3D>>());

        for (auto &line : lines)
        {
            Eigen::Vector3f newDir = transform().rotation() * line.direction();
            Eigen::Vector3f newOrigin = transform() * line.origin();
            line = RPIMoCap::Line3D(newOrigin, newDir);
        }

        emit linesReceived(m_id, lines);
    }

    void onPointsDataReceived(const QByteArray &data)
    {
        msgpack::object_handle result;
        msgpack::unpack(result, data.data(), data.length());

        const std::vector<cv::Point2f> points(result.get().as<std::vector<cv::Point2f>>());
        emit pointsReceived(m_id, points);
    }

private:
    mutable std::mutex m_transformMutex;

    int m_id = -1;
    RPIMoCap::MQTTSubscriber m_lineSub;
    RPIMoCap::MQTTSubscriber m_pointSub;
    Eigen::Affine3f m_transform;
};
