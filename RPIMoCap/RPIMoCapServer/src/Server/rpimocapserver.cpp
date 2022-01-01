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

#include "Server/rpimocapserver.h"
#include "Server/Calibration/floorcalibration.h"

#include <RPIMoCap/Core/avahibrowser.h>
#include <RPIMoCap/Core/topics.h>

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

    for (const auto& cam : qAsConst(m_clients))
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

void Server::onCalibStart(bool start, WandCalibration::Settings settings)
{
    m_aggregator.startCalib(start, settings);
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

/**
     * @brief converts between quaternion representation to euler angels.
     *
     * @param q Quaternion
     * @return Vector3[roll, pitch, yaw] in rad
     *
     * @note Pitch is only limited to +/-90° of freedom.
     *
     * source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_Angles_Conversion
     */
static Eigen::Vector3f toEulerAngles(const Eigen::Quaternionf& q) {
    // pitch (X-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    double pitch = std::atan2(sinr_cosp, cosr_cosp);

    // yaw (Y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    double yaw = std::abs(sinp) >= 1 ? std::copysign(M_PI / 2, sinp) : std::asin(sinp); // use 90 degrees if out of range

    // roll (Z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    double roll = std::atan2(siny_cosp, cosy_cosp);

    return {pitch, yaw, roll};
}


void Server::calibrateFloor(const float offset)
{
    std::vector<FloorCalibration::CameraData> inputData;

    // convert data from last frame
    const Frame lastFrame = m_aggregator.lastFrame();

    for (const auto& observation : lastFrame.observations()) {

        std::vector<cv::Point2f> detectedPixels;
        std::vector<Line3D> rays;

        for (const auto &data : observation.second) { // TODO vector of structures vs structure of vectors in whole system
            detectedPixels.push_back(data.px);
            rays.push_back(data.ray);
        }

        FloorCalibration::CameraData camData{observation.first, m_clients[observation.first]->intrinsics(),
                                             detectedPixels, rays};
        inputData.push_back(camData);
    }

    // get result of calibration
    auto transform = FloorCalibration::computeOrigin(inputData, 30.0f); //TODO size from settings

    // apply result to all cameras
    if (!transform.has_value()) {
        qWarning() << "Floor calibration failed";
        return;
    }

    for (auto& client : m_clients) {
        const Eigen::Affine3f newTransform = transform.value().inverse() * client->transform();

        auto trans = newTransform.translation().cast<double>();
        auto rot = toEulerAngles(Eigen::Quaternionf(newTransform.rotation())).cast<double>();

        client->setTranslation(trans);
        client->setRotation(rot);
    }

}

void Server::setupMQTT(const MQTTSettings &settings)
{
    const QString uuid = QUuid::createUuid().toString(QUuid::StringFormat::WithoutBraces);
    m_triggerPub = std::make_unique<RPIMoCap::MQTTPublisher<std::string>>("serverTriggerPub-" + uuid, RPIMoCap::MQTTTopics::trigger, settings);
}

}
