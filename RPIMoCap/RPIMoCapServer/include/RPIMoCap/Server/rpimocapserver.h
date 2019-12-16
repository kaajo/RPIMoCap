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

#include "linesaggregator.h"
#include "camerasettings.h"

#include <RPIMoCap/Core/mqttpublisher.h>
#include <RPIMoCap/Core/mqttsubscriber.h>

#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <QHash>
#include <QProcess>

#include <memory>

class RPIMoCapServer : public QObject
{
    Q_OBJECT
public:
    explicit RPIMoCapServer(RPIMoCap::MQTTSettings settings, QObject *parent = nullptr);
    virtual ~RPIMoCapServer();

signals:
    void linesReceived(const std::vector<RPIMoCap::Line3D> &lines);
    void pointsReceived(const std::vector<cv::Point2i> &points);

    void cameraAdded(const std::shared_ptr<CameraSettings> &settings);
    void cameraRemoved(int id);

public slots:
    void onMoCapStart(bool start);
    void onCalibStart(bool start);

private slots:
    void onNewConnection();
    void onLostConnection();

private:
    int nextId = 1;
    QTcpServer m_tcpServer;
    QHash<QTcpSocket*, std::shared_ptr<CameraSettings>> m_currentClients;

    void addClient(QTcpSocket *conn, const int id);

    LinesAggregator m_aggregator;
    RPIMoCap::MQTTPublisher<std::string> m_triggerPub;

    QProcess m_avahiPublish;
};
