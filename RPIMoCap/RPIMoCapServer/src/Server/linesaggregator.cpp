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

#include "RPIMoCap/Server/linesaggregator.h"

#include <QtDebug>
#include <QThread>

#include <msgpack.hpp>

#include <chrono>

namespace RPIMoCap {

LinesAggregator::LinesAggregator(QObject *parent) : QObject(parent)
{

}

void LinesAggregator::startCalib()
{
    auto params = Camera::Intrinsics::computeRPICameraV1Params();
    m_wandCalib = std::make_unique<WandCalibration>(m_clients,params);
}

void LinesAggregator::stopCalib()
{
    m_wandCalib.reset();
}

void LinesAggregator::addCamera(const std::shared_ptr<CameraSettings> &camera)
{
    connect(camera.get(),&CameraSettings::raysReceived, this, &LinesAggregator::onRaysReceived);

    if (!m_clients.contains(camera->id()))
    {
        m_clients.insert(camera->id(), camera);
        m_framesReceived.insert(camera->id(), false);
    }
}

void LinesAggregator::removeCamera(const QUuid id)
{
    if (m_clients.contains(id))
    {
        disconnect(m_clients[id].get(),&CameraSettings::raysReceived, this, &LinesAggregator::onRaysReceived);

        m_clients.remove(id);
        m_framesReceived.remove(id);
    }
}

void LinesAggregator::onMoCapStart(bool start)
{
    for (auto &received : m_framesReceived)
    {
        received = false;
    }

    running = start;
    if (start)
    {
        emit trigger();
    }
}

void LinesAggregator::onRaysReceived(const QUuid clientId, const std::vector<std::pair<cv::Point2f, Line3D>> &rays)
{
    if (m_clients.find(clientId) == m_clients.end())
    {
        return;
    }

    m_currentRays[clientId] = rays;
    m_framesReceived[clientId] = true;

    bool haveAll = true;

    for (auto &received : m_framesReceived)
    {
        if (!received)
        {
            haveAll = false;
        }
    }

    //qDebug() << QTime::currentTime().toString("hh:mm:ss.zzz") << rays.size() << "rays received from " << clientId;

    if (running && haveAll)
    {
        if (m_wandCalib)
        {
            std::vector<std::pair<QUuid, std::vector<cv::Point2f>>> pixels;
            pixels.reserve(m_currentRays.size());

            for (auto &id : m_currentRays.keys())
            {
                std::vector<cv::Point2f> camPixels;
                for (auto &ray : m_currentRays[id])
                {
                    camPixels.push_back(ray.first);
                }
                pixels.push_back({id,camPixels});
            }

            m_wandCalib->addFrame(pixels);
        }

        for (auto &received : m_framesReceived)
        {
            received = false;
        }

        auto curTime = QTime::currentTime();

        //qDebug() << curTime.toString("hh:mm:ss.zzz") << "ms elapsed: " << lastTime.msecsTo(curTime);
        lastTime = curTime;

        std::vector<RPIMoCap::Frame::LineSegment> frameLines;

        for (auto &idRays : m_currentRays)
        {
            for (auto &ray : idRays)
            {
                frameLines.push_back({300, ray.second});
            }
        }

        RPIMoCap::Frame frame(std::chrono::high_resolution_clock::now(), frameLines);
        emit frameReady(frame);
        emit trigger();
    }
}

}
