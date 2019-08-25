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

#include "rpimocapclient.h"
#include "avahibrowser.h"

#include <QTimerEvent>

#include <chrono>

RPIMoCapClient::RPIMoCapClient(cv::Size2f cameraFoVRad, QObject *parent)
    : QObject(parent)
    , m_camera("v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=90/1 ! videoconvert ! video/x-raw,format=GRAY8 ! appsink max-buffers=1 name=appsink")
    , m_markerDetector(cameraFoVRad)
{
    qDebug() << "starting RPIMoCapClient, camera FoV" << cameraFoVRad.width * 180.0f/M_PI
             << "x" << cameraFoVRad.height * 180.0f/M_PI;

    connect(&m_rpimocaptcp, &QTcpSocket::readyRead,this,&RPIMoCapClient::onTcpMessage);
    connect(&m_rpimocaptcp, &QTcpSocket::disconnected, this, &RPIMoCapClient::onTcpDisconnected);

    m_avahiCheckTimerID = QObject::startTimer(5000);
}

RPIMoCapClient::~RPIMoCapClient()
{
    mosqpp::lib_cleanup();
}

void RPIMoCapClient::onLines(const std::vector<RPIMoCap::Line3D> &lines)
{
    std::stringstream buf;
    msgpack::pack(buf, lines);
    emit linesSerialized(QByteArray::fromStdString(buf.str()));
}

void RPIMoCapClient::trigger()
{
    if (!m_camera.getOpened()) {
        if (!m_camera.open()) {
            qCritical() << "cannot open camera";
            return;
        }
    }

    auto start = std::chrono::high_resolution_clock::now();

    cv::Mat currentImage = m_camera.pullData();
    //cv::imwrite("/tmp/image.jpg",currentImage);

    if (currentImage.empty())
    {
        qDebug() << "empty image from gst";
    }

    onLines(m_markerDetector.onImage(currentImage));

    auto end = std::chrono::high_resolution_clock::now();

    qDebug() << "elapsed: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
}

void RPIMoCapClient::onTcpMessage()
{
    const QByteArray id = m_rpimocaptcp.readAll();
    qDebug() << "ID assigned:" << QString(id);
    initMQTT(QString(id).toInt());
}

void RPIMoCapClient::onTcpDisconnected()
{
    m_cameraTriggerSub.reset();
    m_linePub.reset();

    m_avahiCheckTimerID = QObject::startTimer(5000);

    qDebug() << "TCP disconnect";
}

void RPIMoCapClient::timerEvent(QTimerEvent *event)
{
    if (event->timerId() != m_avahiCheckTimerID)
    {
        return;
    }

    checkAvahiServices();
}

void RPIMoCapClient::checkAvahiServices()
{
    const auto services = AvahiBrowser::browseServices(AvahiBrowser::IPVersion::IPv4);
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

bool RPIMoCapClient::isMQTTInitialized()
{
    return m_cameraTriggerSub && m_linePub;
}

void RPIMoCapClient::initMQTT( const int32_t cameraid)
{
    if (isMQTTInitialized())
    {
        return;
    }

    m_cameraTriggerSub = std::make_shared<RPIMoCap::MQTTSubscriber>("triggersub" + QString::number(cameraid), "/trigger", m_MQTTsettings);
    m_linePub = std::make_shared<RPIMoCap::MQTTPublisher>("linespub" + QString::number(cameraid),"/client" + QString::number(cameraid) + "/lines",m_MQTTsettings);

    connect(m_cameraTriggerSub.get(), &RPIMoCap::MQTTSubscriber::messageReceived,this, &RPIMoCapClient::trigger);
    connect(this, &RPIMoCapClient::linesSerialized, m_linePub.get(), &RPIMoCap::MQTTPublisher::publishData);
}
