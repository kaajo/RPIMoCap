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

#include <QLoggingCategory>

#include <string>

Q_DECLARE_LOGGING_CATEGORY(MQTT)

namespace RPIMoCap {

enum class MQTTQoS : int {
    AtMostOnce = 0,
    AtLeastOnce = 1,
    ExactlyOnce = 2
};

struct MQTTSettings
{
    std::string IPAddress = "127.0.0.1";
    int port = 1883;
    MQTTQoS QoS = MQTTQoS::ExactlyOnce;
};

}

