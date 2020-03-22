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

#include "RPIMoCap/Core/avahibrowser.h"

#include <QProcess>

using NetworkLayerProtocol = QAbstractSocket::NetworkLayerProtocol;

namespace RPIMoCap {

QVector<AvahiBrowser::ServiceInfo> AvahiBrowser::browseServices(const NetworkLayerProtocol &searchIPVersion) {
    QProcess avahiBrowser;
    avahiBrowser.start("avahi-browse", QStringList() << ("-atpr"));
    avahiBrowser.waitForFinished();

    const QStringList lines = QString(avahiBrowser.readAll()).split("\n",QString::SplitBehavior::SkipEmptyParts);

    QVector<ServiceInfo> services;
    for (const QString &line : lines)
    {
        const QStringList d = line.split(";");
        if (d.first() == "=")
        {
            const auto serviceIPVer = d[2] == "IPv4" ? NetworkLayerProtocol::IPv4Protocol : NetworkLayerProtocol::IPv6Protocol;

            if (searchIPVersion == serviceIPVer || searchIPVersion == NetworkLayerProtocol::AnyIPProtocol)
            {
                QString desc = d[9];

                if (desc.size() > 1)
                {
                    desc = desc.remove(0,1).chopped(1);
                }

                services.push_back({d[1],searchIPVersion, d[4], d[5], QHostAddress(d[7]), d[8].toShort(), desc});
            }
        }
    }
    return services;
}

}
