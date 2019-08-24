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

#include <mqttpublisher.h>
#include <mqttsubscriber.h>

#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <QHash>

#include <memory>

struct ClientData
{
    ClientData(const int id, const RPIMoCap::MQTTSettings &settings)
        : id(id)
        , lineSub(new RPIMoCap::MQTTSubscriber("serverLinesSub","/client" + QString::number(id) + "/lines",settings))
    {

    }

    int id = -1;
    std::shared_ptr<RPIMoCap::MQTTSubscriber> lineSub;
};

class RPIMoCapServer : public QObject
{
    Q_OBJECT
public:
    explicit RPIMoCapServer(QObject *parent = nullptr);

signals:
    void linesReceived(const std::vector<RPIMoCap::Line3D> &lines);

public slots:
    void onMoCapStart(bool start);

private slots:
    void onNewConnection();
    void onLostConnection();

private:
    int nextId = 1;
    QTcpServer m_tcpServer;
    QHash<QTcpSocket*,ClientData> m_currentClients;

    void addClient(QTcpSocket *conn, const int id);

    LinesAggregator m_aggregator;
    std::shared_ptr<RPIMoCap::MQTTPublisher> m_triggerPub;
};
