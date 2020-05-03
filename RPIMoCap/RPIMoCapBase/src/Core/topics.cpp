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

#include "RPIMoCap/Core/topics.h"

namespace RPIMoCap::MQTTTopics {

QString uuidString(const QUuid &uuid)
{
    return uuid.toString(QUuid::StringFormat::WithoutBraces);
}

QString clientPrefix(const QUuid &uuid)
{
    return "/client-" + uuidString(uuid);
}

QString pixels(const QUuid &uuid)
{
    return clientPrefix(uuid) + "/pixels";
}

}
