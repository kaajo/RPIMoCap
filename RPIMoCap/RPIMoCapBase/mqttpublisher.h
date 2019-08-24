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

namespace RPIMoCap {

class MQTTPublisher : public QObject, protected mosqpp::mosquittopp
{
    Q_OBJECT
public:
    MQTTPublisher(QString clientName, QString topic, MQTTSettings settings, QObject *parent = nullptr);

public slots:
    void publishData(const QByteArray &data);

private:
    void on_connect(int rc) override;
    void on_disconnect(int rc) override;
    void on_log(int log_level, const char *str) override;
    void on_error() override;

    QString m_clientName;
    QString m_topic;
    MQTTSettings m_settings;
};

}
