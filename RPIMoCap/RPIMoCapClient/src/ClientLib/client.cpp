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

#include <QTimerEvent>
#include <QJsonDocument>

#include <chrono>

namespace RPIMoCap {

Client::Client(std::shared_ptr<ICamera> camera,
               CameraParams camParams, QObject *parent)
    : QObject(parent)
    , m_camera(camera)
    , m_markerDetector({})
    , m_avahiPublish(this)
{
    //qDebug() << "starting RPIMoCapClient, camera FoV" << camParams.cameraMatrix.at<float>() * 180.0f/M_PI
    //         << "x" << camParams.height * 180.0f/M_PI;

    QString type = "_rpimocap._tcp";
    QString port = QString::number(5000);
    QString idString = m_clientID.toString(QUuid::StringFormat::WithoutBraces);

    QVariantMap data;
    data["id"] = idString;
    data["imageWidth"] = camParams.imageSize.width;
    data["imageHeight"] = camParams.imageSize.height;

    QString desc = QJsonDocument::fromVariant(data).toJson(QJsonDocument::JsonFormat::Compact);
    QStringList params = {"RPIMoCap-Client-" + idString, type, port, desc};

    m_avahiPublish.start("avahi-publish-service", params);

    checkAvahiServices();
    initMQTT(idString);
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

void Client::cameraTrigger()
{
    if (!m_camera->getOpened()) {
        if (!m_camera->open()) {
            qCritical() << "cannot open camera";
            return;
        }
    }

    //auto start = std::chrono::high_resolution_clock::now();

    cv::Mat currentImage = m_camera->pullData();
    //cv::imwrite("/tmp/image.jpg",currentImage);

    if (currentImage.empty())
    {
        qDebug() << "empty image from camera";
    }

    //std::vector<Line3D> lines;
    std::vector<cv::Point2f> points = m_markerDetector.detectMarkers(currentImage);

    m_pointPub->publishData(points);

    //auto end = std::chrono::high_resolution_clock::now();

    //qDebug() << "elapsed: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
}

void Client::timerEvent(QTimerEvent *event)
{
    if (event->timerId() != m_avahiCheckTimerID)
    {
        return;
    }

    checkAvahiServices();
}

void Client::checkAvahiServices()
{
    if (!isMQTTInitialized())
    {
        const auto services = AvahiBrowser::browseServices(QAbstractSocket::NetworkLayerProtocol::IPv4Protocol);
        const auto mqttService = services.find("_mqtt._tcp");

        if (mqttService == services.end())
        {
            qDebug() << "no MQTT service available on local network";
            return;
        }
        else
        {
            m_MQTTsettings.IPAddress = mqttService.value().ipAddress.toString().toStdString();
            m_MQTTsettings.port = mqttService.value().port;
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

void Client::initMQTT(const QString &idString)
{
    if (isMQTTInitialized())
    {
        return;
    }

    m_cameraTriggerSub = std::make_shared<MQTTSubscriber>("triggersub-" + idString, "/trigger", m_MQTTsettings);
    m_pointPub = std::make_shared<MQTTPublisher<std::vector<cv::Point2f>>>("pointspub-" + idString,"/client-" + idString + "/points", m_MQTTsettings);
    connect(m_cameraTriggerSub.get(), &MQTTSubscriber::messageReceived,this, &Client::cameraTrigger);
}

}
