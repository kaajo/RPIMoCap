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

#include "RPIMoCap/Server/rpimocapserver.h"

RPIMoCapServer::RPIMoCapServer(QObject *parent)
    : QObject(parent)
{
    m_avahiPublish.start("avahi-publish-service", {"RPIMoCap Server", "_rpimocap._tcp", QString::number(5000), "RPIMoCap service"});

    connect(&m_aggregator, &LinesAggregator::linesReceived, this, &RPIMoCapServer::linesReceived); //TODO intersector, TODO tracker (pipeline)

    m_tcpServer.listen(QHostAddress::Any,5000);
    connect(&m_tcpServer,&QTcpServer::newConnection,this,&RPIMoCapServer::onNewConnection);

    RPIMoCap::MQTTSettings settings;
    settings.IPAddress = "127.0.0.1";
    m_triggerPub = std::make_shared<RPIMoCap::MQTTPublisher>("serverTriggerPub", "/trigger", settings);
    connect(&m_aggregator,&LinesAggregator::trigger,m_triggerPub.get(),&RPIMoCap::MQTTPublisher::publishData);
}

RPIMoCapServer::~RPIMoCapServer()
{
    m_avahiPublish.kill();
}

void RPIMoCapServer::onMoCapStart(bool start)
{
    m_aggregator.onMoCapStart(start);
}

void RPIMoCapServer::onNewConnection()
{
    const int thisID = nextId;
    QTcpSocket *conn = m_tcpServer.nextPendingConnection();

    connect(conn,&QTcpSocket::disconnected,this,&RPIMoCapServer::onLostConnection);

    conn->write(QString::number(thisID).toUtf8());
    ++nextId;

    addClient(conn,thisID);
}

void RPIMoCapServer::onLostConnection()
{
    QTcpSocket *socket = qobject_cast<QTcpSocket*>(sender());
    const auto it = m_currentClients.find(socket);

    if (it != m_currentClients.end())
    {
        m_aggregator.removeCamera(it.value()->id());
        emit cameraRemoved(it.value()->id());
        m_currentClients.erase(it);
    }
}

void RPIMoCapServer::addClient(QTcpSocket *conn, const int id)
{
    RPIMoCap::MQTTSettings settings;
    settings.IPAddress = "127.0.0.1";

    auto camera = std::make_shared<CameraSettings>(id, settings);
    m_currentClients.insert(conn,camera);

    connect(camera.get(),&CameraSettings::linesReceived, &m_aggregator, &LinesAggregator::onLinesReceived);

    m_aggregator.addCamera(camera);

    emit cameraAdded(camera);
}

