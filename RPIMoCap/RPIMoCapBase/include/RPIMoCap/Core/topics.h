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
#include <QUuid>

#include <string>

namespace RPIMoCap::MQTTTopics {

/**
 * @brief SW trigger for cameras - all clients must listen to this topic
 */
static QString trigger = "/trigger";

/**
 * @brief Function for converting QUuid to QString. Guarantees same conversion on both sides (clients and server)
 * @param Uuid of client
 * @return Converted QString
 */
QString uuidString(const QUuid& uuid);

/**
 * @brief Prefix for all client topics
 * @param uuid
 * @return
 */
QString clientPrefix(const QUuid& uuid);

/**
 * @brief The topic for publishing detected centers of markers
 * @param uuid
 * @return
 */
QString pixels(const QUuid& uuid);

}
