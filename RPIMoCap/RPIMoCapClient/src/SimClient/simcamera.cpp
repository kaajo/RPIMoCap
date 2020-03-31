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

#include "RPIMoCap/SimClient/simcamera.h"

#include <QThread>
#include <QDebug>

namespace RPIMoCap::SimClient {

SimCamera::SimCamera(const Camera::Intrinsics &params, const SimScene &scene)
    : m_params(params)
    , m_scene(scene)
{
    m_timer.start();
}

bool SimCamera::open()
{
    m_opened = true;
    return m_opened;
}

void SimCamera::close()
{
    m_opened = false;
}

cv::Mat SimCamera::pullData()
{
    const int64_t waitTime = 1000.0/m_params.maxFPS - m_timer.restart();

    if (waitTime > 0)
    {
        QThread::msleep(waitTime);
        m_timer.restart();
    }

    return m_scene.projectScene(m_params, m_rotation, m_translation);
}

Camera::Intrinsics &SimCamera::getParams()
{
    return m_params;
}

cv::Vec3f SimCamera::getTranslation() const
{
    return m_translation;
}

void SimCamera::setTranslation(const cv::Vec3f &translation)
{
    m_translation = translation;
}

cv::Vec3f SimCamera::getRotation() const
{
    return m_rotation;
}

void SimCamera::setRotation(const cv::Vec3f &rotation)
{
    m_rotation = rotation;
}

}
