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

#include "icamera.h"
#include "markerdetector.h"

#include <RPIMoCap/Core/line3d.h>
#include <RPIMoCap/Core/msgpack_defs.h>
#include <RPIMoCap/Core/mqttsubscriber.h>
#include <RPIMoCap/Core/mqttpublisher.h>
#include <RPIMoCap/Core/cameraparams.h>

#include <QObject>
#include <QTcpSocket>
#include <QProcess>
#include <QUuid>

#include <msgpack/pack.h>

namespace RPIMoCap {

class Client : public QObject
{
    Q_OBJECT
public:
    explicit Client(std::shared_ptr<ICamera> camera,
                    CameraParams camParams, QObject *parent = nullptr);
    ~Client();

    QUuid id() {return m_clientID;}

signals:
    void error(std::string error);

public slots:
    void cameraTrigger();

private:
    virtual void timerEvent(QTimerEvent *event) override;

    std::shared_ptr<ICamera> m_camera;
    MarkerDetector m_markerDetector;

    std::optional<int> m_avahiCheckTimerID = std::nullopt;
    void checkAvahiServices();

    bool isMQTTInitialized();
    void initMQTT(const QString &idString);
    MQTTSettings m_MQTTsettings;
    std::shared_ptr<MQTTSubscriber> m_cameraTriggerSub;
    std::shared_ptr<MQTTPublisher<std::vector<cv::Point2f>>> m_pointPub;

    QUuid m_clientID = QUuid::createUuid();
    QProcess m_avahiPublish;
};

}
