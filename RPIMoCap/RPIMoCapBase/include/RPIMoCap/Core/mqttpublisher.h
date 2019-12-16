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

#include "mqttsettings.h"

#include <QByteArray>
#include <QObject>

#include <mosquittopp.h>
#include <msgpack/pack_decl.hpp>
#include <msgpack/sbuffer.hpp>

namespace RPIMoCap {

template<class Data>
class MQTTPublisher : protected mosqpp::mosquittopp
{
public:
    MQTTPublisher(QString clientName, QString topic, MQTTSettings settings)
        : mosqpp::mosquittopp(clientName.toStdString().c_str())
        , m_clientName(std::move(clientName))
        , m_topic(std::move(topic))
        , m_settings(std::move(settings))
    {
        auto conRes = connect_async(m_settings.IPAddress.c_str(),m_settings.port);
        switch(conRes) {
        case MOSQ_ERR_INVAL:
            qCDebug(MQTT) << m_clientName << QString("the input parameters were invalid");
            break;
        case MOSQ_ERR_SUCCESS:
            loop_start();
            break;
        default:
            qCCritical(MQTT) << m_clientName << mosquitto_strerror(conRes);
            break;
        }
    }

    void publishData(const Data &data)
    {
        msgpack::sbuffer ss;
        msgpack::pack(ss, data);
        mosqpp::mosquittopp::publish(nullptr, m_topic.toStdString().c_str(), static_cast<int>(ss.size()), ss.data(),
                                     static_cast<std::underlying_type_t<MQTTQoS>>(m_settings.QoS), false);
    }

private:
    void on_connect(int rc) override {
        switch(rc) {
        case MOSQ_ERR_INVAL:
            qCDebug(MQTT) << m_clientName << QString("the input parameters were invalid");
            break;
        case MOSQ_ERR_SUCCESS:
            loop_start();
            break;
        default:
            qCCritical(MQTT) << m_clientName << mosquitto_strerror(rc);
            break;
        }
    }

    void on_disconnect(int rc) override {
        qCCritical(MQTT) << m_clientName << mosquitto_strerror(rc);
    }

    void on_log(int log_level, const char *str) override {
        switch (log_level) {
        case MOSQ_LOG_DEBUG:
            qCDebug(MQTT) << m_clientName << str;
            break;
        case MOSQ_LOG_INFO:
        case MOSQ_LOG_NOTICE:
        case MOSQ_LOG_SUBSCRIBE:
        case MOSQ_LOG_UNSUBSCRIBE:
            qCInfo(MQTT) << m_clientName << str;
            break;
        case MOSQ_LOG_WARNING:
            qCWarning(MQTT) << m_clientName << str;
            break;
        case MOSQ_LOG_ERR:
            qCCritical(MQTT) << m_clientName << str;
            break;
        default:
            qCDebug(MQTT) << m_clientName << str;
            break;
        }
    }

    void on_error() override {
        qCCritical(MQTT) << m_clientName << "error";
    }

    QString m_clientName;
    QString m_topic;
    MQTTSettings m_settings;
};

}
