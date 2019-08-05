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
#include <QDebug>

#include <mosquittopp.h>

namespace RPIMoCap {

class MQTTSubscriber : public QObject, protected mosqpp::mosquittopp
{
    Q_OBJECT
public:
    MQTTSubscriber(std::string clientName, std::string topic, MQTTSettings settings, QObject *parent = nullptr);

signals:
    void messageReceived(const QByteArray &data);

private:
    void on_connect(int rc) override;
    void on_message(const mosquitto_message *message) override;
    void on_log(int, const char *str) override;
    void on_error() override;

    std::string m_clientName;
    std::string m_topic;
    MQTTSettings m_settings;
};

}
