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

namespace RPIMoCap::SimClient {

SimCamera::SimCamera(const CameraParams &params, const SimScene &scene)
    : m_params(params)
    , m_scene(scene)
{

}

bool SimCamera::open()
{
    return true;
}

void SimCamera::close()
{

}

cv::Mat SimCamera::pullData()
{
    return m_scene.projectScene(m_params);
}

}
