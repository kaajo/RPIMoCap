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

RPIMoCapServer::RPIMoCapServer(RPIMoCap::MQTTSettings settings, QObject *parent)
    : QObject(parent)
    , m_triggerPub("serverTriggerPub", "/trigger", settings)
{
    connect(&m_aggregator,&LinesAggregator::trigger, [&](){m_triggerPub.publishData("");});
}

RPIMoCapServer::~RPIMoCapServer()
{
}

void RPIMoCapServer::onMoCapStart(bool start)
{
    m_aggregator.onMoCapStart(start);
}

void RPIMoCapServer::onCalibStart(bool start)
{
    start ? m_aggregator.startCalib() : m_aggregator.stopCalib();
}

void RPIMoCapServer::searchForCameras()
{
    auto services = RPIMoCap::AvahiBrowser::browseServices(QAbstractSocket::IPv4Protocol);

    RPIMoCap::MQTTSettings settings;
    settings.IPAddress = "127.0.0.1";

    for (auto &service : services)
    {
        if (service.type == "_rpimocap._tcp")
        {
            const QJsonDocument json = QJsonDocument::fromJson(service.description.toUtf8());
            const QVariantMap descVar = json.toVariant().toMap();

            QUuid id(descVar["id"].toString());

            auto camera = std::make_shared<CameraSettings>(id, settings);
            emit cameraAdded(camera);

            connect(camera.get(),&CameraSettings::pointsReceived, &m_aggregator, &LinesAggregator::onPointsReceived);

            m_aggregator.addCamera(camera);
        }
    }
}
