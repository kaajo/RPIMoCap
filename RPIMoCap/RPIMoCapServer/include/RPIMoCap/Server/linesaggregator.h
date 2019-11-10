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

#include <RPIMoCap/Server/wandcalibration.h>
#include <RPIMoCap/Server/camerasettings.h>

#include <RPIMoCap/Core/line3d.h>
#include <RPIMoCap/Core/frame.h>

#include <QObject>
#include <QTime>
#include <QMap>

class LinesAggregator : public QObject
{
    Q_OBJECT
public:
    explicit LinesAggregator(QObject *parent = nullptr);

    void addCamera(const std::shared_ptr<CameraSettings> &camera);
    void removeCamera(const int id);

    void startCalib();
    void stopCalib();

signals:
    void trigger(const QByteArray &payload);

    /**
     * @brief When lines are received from all cameras, new RPIMoCap::Frame with proper time and lines is emitted
     * @param frame
     */
    void frameReady(const RPIMoCap::Frame &frame);
    void linesReceived(const std::vector<RPIMoCap::Line3D> &lines);

public slots:
    void onMoCapStart(bool start);
    void onLinesReceived(const int clientId, const std::vector<RPIMoCap::Line3D> &lines);
    void onPointsReceived(const int clientId, const std::vector<cv::Point2i> &points);

private:
    QTime lastTime = QTime::currentTime();
    bool running = false;

    QMap<int,std::shared_ptr<CameraSettings>> m_clients;

    QVector<RPIMoCap::Line3D> m_currentlines;
    QMap<int,bool> m_currentlyReceived;

    std::unique_ptr<WandCalibration> m_wandCalib;
    QMap<int,std::vector<cv::Point2f>> m_currentPoints;
    QMap<int,bool> m_currentlyReceivedPoints;
};

