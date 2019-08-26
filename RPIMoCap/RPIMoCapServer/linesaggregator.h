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

#include <line3d.h>

#include <QObject>
#include <QTime>

class LinesAggregator : public QObject
{
    Q_OBJECT
public:
    explicit LinesAggregator(QObject *parent = nullptr);

signals:
    void trigger(const QByteArray &payload);
    void linesReceived(const std::vector<RPIMoCap::Line3D> &lines);

public slots:
    void onMoCapStart(bool start);
    void onLinesReceived(const int clientId, const std::vector<RPIMoCap::Line3D> &lines);

private:
    QTime lastTime = QTime::currentTime();
    bool running = false;
};

