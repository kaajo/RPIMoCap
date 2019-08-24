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

#include "rpicamera.h"
#include "markerdetector.h"

#include <line3d.h>

#include <QObject>
#include <QByteArray>
#include <QHostAddress>
#include <QNetworkInterface>
#include <QTcpSocket>

#include <msgpack/pack.h>
#include <mqttsubscriber.h>
#include <mqttpublisher.h>

class RPIMoCapClient : public QObject
{
    Q_OBJECT
public:
    explicit RPIMoCapClient(cv::Size2f cameraFoVRad, QObject *parent = nullptr);
    ~RPIMoCapClient();

signals:
    void error(std::string error);
    void linesSerialized(const QByteArray &lines);

public slots:
    void onLines(const std::vector<RPIMoCap::Line3D> &lines);
    void trigger();

    void onTcpMessage();

private:
    bool opened = false;
    GstCVCamera m_camera;
    MarkerDetector m_markerDetector;

    QTcpSocket m_rpimocaptcp;

    bool isMQTTInitialized();
    void initMQTT(const int32_t cameraid);
    RPIMoCap::MQTTSettings m_MQTTsettings;
    std::shared_ptr<RPIMoCap::MQTTSubscriber> m_cameraTriggerSub;
    std::shared_ptr<RPIMoCap::MQTTPublisher> m_linePub;
};
