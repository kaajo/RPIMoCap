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

#include <RPIMoCapBase/mqttsubscriber.h>
#include <RPIMoCapBase/line3d.h>

#include <QObject>

#include <eigen3/Eigen/Geometry>
#include <msgpack.hpp>

#include <memory>
#include <mutex>

struct CameraSettings : QObject
{
    Q_OBJECT
public:
    CameraSettings(int id, const RPIMoCap::MQTTSettings &settings, Eigen::Affine3f transform = Eigen::Affine3f::Identity())
        : m_id(id)
        , m_lineSub("serverLinesSub" + QString::number(id),"/client" + QString::number(id) + "/lines",settings)
        , m_transform(transform)
    {
        connect(&m_lineSub,&RPIMoCap::MQTTSubscriber::messageReceived, this, &CameraSettings::onLinesDataReceived);
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

private:
    mutable std::mutex m_transformMutex;

    int m_id = -1;
    RPIMoCap::MQTTSubscriber m_lineSub;
    Eigen::Affine3f m_transform;
};
