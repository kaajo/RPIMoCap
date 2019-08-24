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

#include "rpimocapserver.h"

RPIMoCapServer::RPIMoCapServer(QObject *parent)
    : QObject(parent)
{
    connect(&m_aggregator, &LinesAggregator::linesReceived, this, &RPIMoCapServer::linesReceived);

    m_tcpServer.listen(QHostAddress::Any,5000);
    connect(&m_tcpServer,&QTcpServer::newConnection,this,&RPIMoCapServer::onNewConnection);

    RPIMoCap::MQTTSettings settings;
    settings.IPAddress = "127.0.0.1";
    m_triggerPub = std::make_shared<RPIMoCap::MQTTPublisher>("serverTriggerPub", "/trigger", settings);
    connect(&m_aggregator,&LinesAggregator::trigger,m_triggerPub.get(),&RPIMoCap::MQTTPublisher::publishData);
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
    m_currentClients.remove(socket);
}

void RPIMoCapServer::addClient(QTcpSocket *conn, const int id)
{
    RPIMoCap::MQTTSettings settings;
    settings.IPAddress = "127.0.0.1";

    ClientData clientData(id,settings);
    m_currentClients.insert(conn,clientData);
    connect(clientData.lineSub.get(),&RPIMoCap::MQTTSubscriber::messageReceived,&m_aggregator,&LinesAggregator::onLinesReceived);
}

