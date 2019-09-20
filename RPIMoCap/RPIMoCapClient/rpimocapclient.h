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

#include <RPIMoCapBase/line3d.h>
#include <RPIMoCapBase/msgpack_defs.h>
#include <RPIMoCapBase/mqttsubscriber.h>
#include <RPIMoCapBase/mqttpublisher.h>

#include <QObject>
#include <QByteArray>
#include <QHostAddress>
#include <QNetworkInterface>
#include <QTcpSocket>

#include <msgpack/pack.h>

class RPIMoCapClient : public QObject
{
    Q_OBJECT
public:
    explicit RPIMoCapClient(cv::Size2f cameraFoVRad, QObject *parent = nullptr);
    ~RPIMoCapClient();

signals:
    void error(std::string error);
    void linesSerialized(const QByteArray &lines);
    void pointsSerialized(const QByteArray &points);

public slots:
    void onLines(const std::vector<RPIMoCap::Line3D> &lines);
    void onPoints(const std::vector<cv::Point2i> &points);
    void trigger();

    void onTcpMessage();
    void onTcpDisconnected();

    virtual void timerEvent(QTimerEvent *event) override;

private:
    GstCVCamera m_camera;
    MarkerDetector m_markerDetector;

    int m_avahiCheckTimerID = -1;
    QTcpSocket m_rpimocaptcp;
    void checkAvahiServices();

    bool isMQTTInitialized();
    void initMQTT(const int32_t cameraid);
    RPIMoCap::MQTTSettings m_MQTTsettings;
    std::shared_ptr<RPIMoCap::MQTTSubscriber> m_cameraTriggerSub;
    std::shared_ptr<RPIMoCap::MQTTPublisher> m_linePub;
    std::shared_ptr<RPIMoCap::MQTTPublisher> m_pointPub;
};
