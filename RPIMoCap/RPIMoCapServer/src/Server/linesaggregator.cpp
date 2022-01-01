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

#include "Server/linesaggregator.h"

#include <QtDebug>
#include <QThread>

#include <msgpack.hpp>

#include <chrono>

namespace RPIMoCap {

LinesAggregator::LinesAggregator(QObject *parent) : QObject(parent)
{

}

void LinesAggregator::startCalib(bool start, WandCalibration::Settings settings)
{
    //TODO get data from settings
    WandCalibration::InputData inputdata;
    inputdata.wandPoints.push_back({-25.0,0.0,0.0});
    inputdata.wandPoints.push_back({10.0,0.0,0.0});
    inputdata.wandPoints.push_back({25.0,0.0,0.0});
    inputdata.camParams = Camera::Intrinsics::computeRPICameraV1Params();
    inputdata.cameraSettings = m_clients;

    m_wandCalib.startCalib(start, settings, inputdata);
}


void LinesAggregator::addCamera(const std::shared_ptr<CameraSettings> &camera)
{
    connect(camera.get(),&CameraSettings::raysReceived, this, &LinesAggregator::onRaysReceived);

    if (!m_clients.contains(camera->id()))
    {
        m_clients.insert(camera->id(), camera);
        m_framesReceived.insert({camera->id(), false});
    }
}

void LinesAggregator::removeCamera(const QUuid id)
{
    if (m_clients.contains(id))
    {
        disconnect(m_clients[id].get(),&CameraSettings::raysReceived, this, &LinesAggregator::onRaysReceived);

        m_clients.remove(id);
        m_framesReceived.erase(id);
    }
}

void LinesAggregator::onMoCapStart(bool start)
{
    for (auto &received : m_framesReceived)
    {
        received.second = false;
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
        qWarning() << "Received data from unexpected client: " << clientId;
        return;
    }

    m_currentRays[clientId] = rays;
    m_framesReceived[clientId] = true;

    //qDebug() << QTime::currentTime().toString("hh:mm:ss.zzz") << rays.size() << "rays received from " << clientId;

    if (running && haveAllDataForFrame())
    {
        if (m_wandCalib.running())
        {
            std::vector<std::pair<QUuid, std::vector<cv::Point2f>>> pixels;
            pixels.reserve(m_currentRays.size());

            for (auto &id : m_currentRays)
            {
                std::vector<cv::Point2f> camPixels;
                for (auto &ray : m_currentRays[id.first])
                {
                    camPixels.push_back(ray.first);
                }
                pixels.push_back({id.first,camPixels});
            }

            m_wandCalib.addFrame(pixels);
        }

        for (auto &received : m_framesReceived)
        {
            received.second = false;
        }

        auto curTime = QTime::currentTime();

        //qDebug() << curTime.toString("hh:mm:ss.zzz") << "ms elapsed: " << lastTime.msecsTo(curTime);
        lastTime = curTime;


//        ////////////// TEST
//        auto idFirst = m_clients.first()->id();
//        auto idLast = m_clients.last()->id();
//        auto pxNorm = m_clients[idFirst]->normalizeCoords(m_currentRays[idFirst][0].first);

//        auto essentialMatrix = m_clients[idFirst]->essentialMatrix(*m_clients[idLast]);
//        auto epipolarLine = m_clients[idFirst]->epipolarLine(pxNorm, essentialMatrix);

//        // Find closest point to epipolar line
//        double minDist = std::numeric_limits<double>::max();
//        cv::Point2f minDistPixel;
//        for (auto& pixelLine : m_currentRays[idLast]) {
//            auto norm = m_clients[idLast]->normalizeCoords(pixelLine.first);
//            auto dist = epipolarLine.distance(Eigen::Vector2d((double)norm.x,(double) norm.y));

//            if (dist < minDist)
//            {
//                minDist = dist;
//                minDistPixel = pixelLine.first;
//            }
//        }

//        std::cout << minDist << std::endl;
//        std::cout << minDistPixel << std::endl;

        //transform lines as needed TODO worth refactoring?
        std::vector<std::pair<QUuid, std::vector<Line3D>>> linesTransformed;
        for (auto it = m_currentRays.begin(); it != m_currentRays.end(); ++it)
        {
            std::vector<Line3D> rayVec;

            std::transform(it->second.begin(), it->second.end(),
                           std::back_inserter(rayVec), [](auto &pair){return pair.second;});

            linesTransformed.push_back({it->first, rayVec});
        }

        //all intersections
        double maxDistancecm = 2.0;
        std::vector<Frame::Marker> results;
        for (size_t i = 0; i < linesTransformed.size(); ++i)
        {
            for (size_t j = 0; j < linesTransformed.size(); ++j)
            {
                if (i == j)
                {
                    continue;
                }

                auto res = stereoIntersections(linesTransformed[i].second, linesTransformed[j].second);

                for (const auto &intersection : res)
                {
                    if (intersection.edgeDistance < maxDistancecm)
                    {
                        results.push_back(Frame::Marker{0, {}, intersection.estimatedPoint}); // TODO construct properly
                    }
                }
            }
        }

        std::unordered_map<QUuid, Frame::CamObservations> observations;

        for (auto &idRays : m_currentRays)
        {
            for (auto &ray : idRays.second)
            {
                observations[idRays.first].push_back(Frame::CamObservation{std::numeric_limits<size_t>::quiet_NaN(), ray.first, ray.second});
            }
        }

        //TODO time of trigger
        RPIMoCap::Frame frame(std::chrono::high_resolution_clock::now(), observations);
        frame.setMarkers(results);

        m_lastFrame = frame;

        emit frameReady(frame);
        emit trigger();
    }
}

bool LinesAggregator::haveAllDataForFrame()
{
    for (auto &received : m_framesReceived)
    {
        if (!received.second)
        {
            return false;
        }
    }

    return true;
}

std::vector<LinesAggregator::TriangulationResult> LinesAggregator::stereoIntersections(const std::vector<Line3D> &lines1, const std::vector<Line3D> &lines2)
{
    std::vector<LinesAggregator::TriangulationResult> retVal;

    for (size_t i = 0; i < lines1.size(); ++i)
    {
        for (size_t j = 0; j < lines2.size(); ++j)
        {
            Eigen::Vector3f iPoint = Eigen::Vector3f::Zero();
            Eigen::Vector3f jPoint = Eigen::Vector3f::Zero();

            if (closestPoints(lines1[i], lines2[j], iPoint, jPoint))
            {
                retVal.push_back({(jPoint - iPoint).norm(), (jPoint + iPoint)/2});
            }
        }
    }

    return retVal;
}

}
