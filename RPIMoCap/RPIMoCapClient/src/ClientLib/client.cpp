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

#include <chrono>

namespace RPIMoCap {

Client::Client(std::shared_ptr<ICamera> camera,
               CameraParams camParams, QObject *parent)
    : QObject(parent)
    , m_camera(camera)
    , m_markerDetector(camParams)
{
    //qDebug() << "starting RPIMoCapClient, camera FoV" << camParams.cameraMatrix.at<float>() * 180.0f/M_PI
    //         << "x" << camParams.height * 180.0f/M_PI;

    connect(&m_rpimocaptcp, &QTcpSocket::readyRead,this,&Client::onTcpMessage);
    connect(&m_rpimocaptcp, &QTcpSocket::disconnected, this, &Client::onTcpDisconnected);

    m_avahiCheckTimerID = QObject::startTimer(5000);
}

Client::~Client()
{
    mosqpp::lib_cleanup();
}

void Client::trigger()
{
    if (!m_camera->getOpened()) {
        if (!m_camera->open()) {
            qCritical() << "cannot open camera";
            return;
        }
    }

    auto start = std::chrono::high_resolution_clock::now();

    cv::Mat currentImage = m_camera->pullData();
    //cv::imwrite("/tmp/image.jpg",currentImage);

    if (currentImage.empty())
    {
        qDebug() << "empty image from camera";
    }

    std::vector<Line3D> lines;
    std::vector<cv::Point2f> points;
    m_markerDetector.onImage(currentImage, lines, points);

    m_linePub->publishData(lines);
    m_pointPub->publishData(points);

    auto end = std::chrono::high_resolution_clock::now();

    qDebug() << "elapsed: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
}

void Client::onTcpMessage()
{
    const QByteArray id = m_rpimocaptcp.readAll();
    qDebug() << "ID assigned:" << QString(id);
    initMQTT(QString(id).toInt());
}

void Client::onTcpDisconnected()
{
    m_cameraTriggerSub.reset();
    m_linePub.reset();

    m_avahiCheckTimerID = QObject::startTimer(5000);

    qDebug() << "TCP disconnect";
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
    const auto services = AvahiBrowser::browseServices(QAbstractSocket::NetworkLayerProtocol::IPv4Protocol);
    const auto rpimocapService = services.find("_rpimocap._tcp");

    if (m_rpimocaptcp.state() != QAbstractSocket::ConnectedState)
    {
        if (rpimocapService == services.end())
        {
            qDebug() << "no RPIMoCap service available on local network";
            return;
        }
        else
        {
            m_rpimocaptcp.connectToHost(rpimocapService.value().ipAddress,rpimocapService.value().port);
            qDebug() << "trying to connect to TCP";
        }
    }

    if (!isMQTTInitialized())
    {
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

    if (m_rpimocaptcp.state() == QAbstractSocket::ConnectedState && isMQTTInitialized())
    {
        QObject::killTimer(m_avahiCheckTimerID);
        m_avahiCheckTimerID = -1;
    }
}

bool Client::isMQTTInitialized()
{
    return m_cameraTriggerSub && m_linePub;
}

void Client::initMQTT( const int32_t cameraid)
{
    if (isMQTTInitialized())
    {
        return;
    }

    const QString cameraIDString = QString::number(cameraid);

    m_cameraTriggerSub = std::make_shared<MQTTSubscriber>("triggersub" + cameraIDString, "/trigger", m_MQTTsettings);
    m_linePub = std::make_shared<MQTTPublisher<std::vector<Line3D>>>("linespub" + cameraIDString,"/client" + cameraIDString + "/lines", m_MQTTsettings);
    m_pointPub = std::make_shared<MQTTPublisher<std::vector<cv::Point2f>>>("pointspub" + cameraIDString,"/client" + cameraIDString + "/points", m_MQTTsettings);

    connect(m_cameraTriggerSub.get(), &MQTTSubscriber::messageReceived,this, &Client::trigger);
}

}
