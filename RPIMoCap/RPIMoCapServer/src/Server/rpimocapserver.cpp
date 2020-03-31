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

#include <RPIMoCap/Core/avahibrowser.h>

#include <QJsonDocument>

namespace RPIMoCap {

Server::Server(QObject *parent)
    : QObject(parent)
{
    connect(&m_aggregator, &LinesAggregator::trigger, this, &Server::trigger);
    connect(this, &Server::cameraAdded, &m_aggregator, &LinesAggregator::addCamera);
    connect(this, &Server::cameraRemoved, &m_aggregator, &LinesAggregator::removeCamera);
    connect(&m_aggregator, &LinesAggregator::frameReady, this, &Server::frameReady);
}

void Server::init()
{
    const auto services = RPIMoCap::AvahiBrowser::browseServices(QAbstractSocket::IPv4Protocol);

    const auto mqttService = std::find_if(services.begin(),services.end(),
                                          [](auto &service){return service.type == "_mqtt._tcp";});

    RPIMoCap::MQTTSettings MQTTsettings;

    if (mqttService == services.end())
    {
        qDebug() << "no MQTT service available on local network";
        return;
    }
    else
    {
        MQTTsettings.IPAddress = mqttService->ipAddress.toString().toStdString();
        MQTTsettings.port = mqttService->port;
        setupMQTT(MQTTsettings);
    }

    for (const auto& cam : m_clients)
    {
        m_aggregator.removeCamera(cam->id());
        emit cameraRemoved(cam->id());
    }

    m_clients.clear();

    for (auto &service : services)
    {
        if (service.type == "_rpimocap._tcp")
        {
            const QJsonDocument json = QJsonDocument::fromJson(service.description.toUtf8());
            const QVariantMap descVar = json.toVariant().toMap();

            const QUuid id(descVar["id"].toString());
            const auto params = Camera::Intrinsics::fromVariantMap(descVar["camParams"].toMap());

            if (!m_clients.contains(id))
            {
                auto camera = std::make_shared<CameraSettings>(id, MQTTsettings, params);
                m_clients[camera->id()] = camera;
                emit cameraAdded(camera);
            }
        }
    }
}

void Server::onCalibStart(bool start)
{
    start ? m_aggregator.startCalib() : m_aggregator.stopCalib();
}

void Server::onMoCapStart(bool start)
{
    m_aggregator.onMoCapStart(start);
}

void Server::trigger()
{
    if (m_triggerPub)
    {
        m_triggerPub->publishData("");
    }
    else
    {
        qWarning() << "Cannot trigger remote cameras - no connection to MQTT";
    }
}

void Server::setupMQTT(const MQTTSettings &settings)
{
    const QString uuid = QUuid::createUuid().toString(QUuid::StringFormat::WithoutBraces);
    m_triggerPub = std::make_unique<RPIMoCap::MQTTPublisher<std::string>>("serverTriggerPub-" + uuid, "/trigger", settings);
}

}
