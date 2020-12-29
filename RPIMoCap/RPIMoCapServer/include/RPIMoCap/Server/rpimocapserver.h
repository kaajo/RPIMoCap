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

#include "linesaggregator.h"
#include "camerasettings.h"

#include <RPIMoCap/Core/mqttpublisher.h>

#include <QObject>

#include <memory>

namespace RPIMoCap {

class Server : public QObject
{
    Q_OBJECT
public:
    explicit Server(QObject *parent = nullptr);
    ~Server() override = default;

signals:
    void cameraAdded(const std::shared_ptr<CameraSettings> &settings);
    void cameraRemoved(QUuid id);
    void frameReady(const Frame &frame);

public slots:
    void init();
    void onCalibStart(bool start, WandCalibration::Settings settings);
    void onMoCapStart(bool start);
    void trigger();

private:
    LinesAggregator m_aggregator;

    void setupMQTT(const RPIMoCap::MQTTSettings &settings);
    std::unique_ptr<RPIMoCap::MQTTPublisher<std::string>> m_triggerPub;

    QMap<QUuid,std::shared_ptr<CameraSettings>> m_clients;
};

}
