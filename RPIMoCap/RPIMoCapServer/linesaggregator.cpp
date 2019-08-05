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

#include "linesaggregator.h"

#include <line3d.h>

#include <QtDebug>
#include <QThread>

#include <msgpack.hpp>

LinesAggregator::LinesAggregator(QObject *parent) : QObject(parent)
{

}

void LinesAggregator::onLinesReceived(const QByteArray &linesData)
{
    //this->thread()->msleep(500);

    msgpack::object_handle result;
    msgpack::unpack(result, linesData.data(), linesData.length());

    std::vector<RPIMoCap::Line3D> lines(result.get().as<std::vector<RPIMoCap::Line3D>>());

    auto curTime = QTime::currentTime();

    qDebug() << "ms elapsed: " << lastTime.msecsTo(curTime) << " lines received: " << lines.size();
    lastTime = curTime;

    emit trigger({});
}
