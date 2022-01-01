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

#include <Server/Calibration/wandcalibration.h>
#include <Server/camerasettings.h>

#include <RPIMoCap/Core/line3d.h>
#include <RPIMoCap/Core/frame.h>

#include <QObject>
#include <QTime>
#include <QMap>

namespace RPIMoCap {

class LinesAggregator : public QObject
{
    Q_OBJECT
public:
    explicit LinesAggregator(QObject *parent = nullptr);

    void startCalib(bool start, WandCalibration::Settings settings);

    Frame lastFrame() const {return m_lastFrame;}

signals:
    void trigger();

    /**
     * @brief When lines are received from all cameras, new RPIMoCap::Frame with proper time and lines is emitted
     * @param frame
     */
    void frameReady(const RPIMoCap::Frame &frame);
    //void linesReceived(const std::vector<RPIMoCap::Line3D> &lines);

public slots:
    void addCamera(const std::shared_ptr<CameraSettings> &camera);
    void removeCamera(const QUuid id);

    void onMoCapStart(bool start);
    void onRaysReceived(const QUuid clientId, const std::vector<std::pair<cv::Point2f, Line3D>> &rays);

    bool haveAllDataForFrame();

private:
    QTime lastTime = QTime::currentTime();
    bool running = false;

    Frame m_lastFrame;

    QMap<QUuid,std::shared_ptr<CameraSettings>> m_clients;

    WandCalibration m_wandCalib;

    std::unordered_map<QUuid,std::vector<std::pair<cv::Point2f, Line3D>>> m_currentRays;
    std::unordered_map<QUuid,bool> m_framesReceived;


    struct TriangulationResult {
        // idea - uncertainty in case points are on epipolar line
        // TODO cameras and rays id
        // TODo number of intersections
        float edgeDistance;
        Eigen::Vector3f estimatedPoint;
    };

    std::vector<TriangulationResult> stereoIntersections(const std::vector<Line3D> &lines1, const std::vector<Line3D> &lines2);
};

}
