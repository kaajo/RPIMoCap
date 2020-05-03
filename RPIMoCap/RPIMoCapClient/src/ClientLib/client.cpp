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

#include "RPIMoCap/ClientLib/client.h"
#include "RPIMoCap/Core/avahibrowser.h"

#include <RPIMoCap/Core/topics.h>

#include <QTimerEvent>
#include <QJsonDocument>

#include <chrono>

namespace RPIMoCap {

Client::Client(std::shared_ptr<ICamera> camera, Camera::Intrinsics camParams,
               const QUuid &id, QObject *parent)
    : QObject(parent)
    , m_camera(camera)
    , m_markerDetector({})
    , m_clientID(id)
    , m_avahiPublish(this)
{
    publishClientService(camParams);
    findMQTTService();
    initMQTT();
}

Client::~Client()
{
    m_avahiPublish.kill();
    m_avahiPublish.waitForFinished(1000);

    if (m_avahiCheckTimerID.has_value())
    {
        QObject::killTimer(m_avahiCheckTimerID.value());
    }
}

QUuid Client::id() const
{
    return m_clientID;
}

void Client::cameraTrigger()
{
    if (!m_camera->getOpened()) {
        if (!m_camera->open()) {
            qCritical() << "cannot open camera";
            return;
        }
    }

    cv::Mat currentImage = m_camera->pullData();

    if (currentImage.empty())
    {
        qDebug() << "empty image from camera";
    }

    std::vector<cv::Point2f> points = m_markerDetector.detectMarkers(currentImage);

    m_pointPub->publishData(points);
}

void Client::timerEvent(QTimerEvent *event)
{
    if (event->timerId() != m_avahiCheckTimerID)
    {
        return;
    }

    findMQTTService();
}

void Client::findMQTTService()
{
    if (!isMQTTInitialized())
    {
        const auto services = AvahiBrowser::browseServices(QAbstractSocket::NetworkLayerProtocol::IPv4Protocol);
        const auto mqttService = std::find_if(services.begin(),services.end(),
                                              [](auto &service){return service.type == "_mqtt._tcp";});

        if (mqttService == services.end())
        {
            qWarning() << "no MQTT service available on local network";
            return;
        }
        else
        {
            m_MQTTsettings.IPAddress = mqttService->ipAddress.toString().toStdString();
            m_MQTTsettings.port = mqttService->port;
        }
    }

    if (isMQTTInitialized() && m_avahiCheckTimerID.has_value())
    {
        QObject::killTimer(m_avahiCheckTimerID.value());
        m_avahiCheckTimerID = std::nullopt;
    }
}

bool Client::isMQTTInitialized()
{
    return m_cameraTriggerSub != nullptr;
}

void Client::initMQTT()
{
    if (isMQTTInitialized())
    {
        return;
    }

    m_cameraTriggerSub = std::make_shared<MQTTSubscriber>("triggersub-" + RPIMoCap::MQTTTopics::uuidString(m_clientID),
                                                          RPIMoCap::MQTTTopics::trigger, m_MQTTsettings);
    m_pointPub = std::make_shared<MQTTPublisher<std::vector<cv::Point2f>>>("pointspub-" + RPIMoCap::MQTTTopics::uuidString(m_clientID),
                                                                           RPIMoCap::MQTTTopics::pixels(m_clientID), m_MQTTsettings);
    connect(m_cameraTriggerSub.get(), &MQTTSubscriber::messageReceived,this, &Client::cameraTrigger);
}

void Client::publishClientService(const Camera::Intrinsics &params)
{
    QString idString = RPIMoCap::MQTTTopics::uuidString(m_clientID);

    QString type = "_rpimocap._tcp";
    QString port = QString::number(5000);

    QVariantMap data;
    data["id"] = idString;
    data["camParams"] = params.toVariantMap();

    QString desc = QJsonDocument::fromVariant(data).toJson(QJsonDocument::JsonFormat::Compact);
    QStringList args = {"RPIMoCap-Client-" + idString, type, port, desc};

    m_avahiPublish.start("avahi-publish-service", args);
}

}
