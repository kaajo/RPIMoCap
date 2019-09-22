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

namespace RPIMoCap {

QHash<QString, AvahiBrowser::ServiceInfo> AvahiBrowser::browseServices(const AvahiBrowser::IPVersion &ipv) {
    QProcess avahiBrowser;
    avahiBrowser.start("avahi-browse", QStringList() << ("-atpr"));
    avahiBrowser.waitForFinished();

    const QStringList lines = QString(avahiBrowser.readAll()).split("\n",QString::SplitBehavior::SkipEmptyParts);

    QHash<QString, ServiceInfo> services;
    for (const QString &line : lines)
    {
        const QStringList d = line.split(";");
        if (d.first() == "=")
        {
            const IPVersion ipver = d[2] == "IPv4" ? IPVersion::IPv4 : IPVersion::IPv6;
            if (ipver == ipv)
            {
                services[d[4]] = ServiceInfo(ServiceInfo{d[1],ipver, d[4], d[5], QHostAddress(d[7]), d[8].toShort(), d[9]});
            }
        }
    }
    return services;
}

}
