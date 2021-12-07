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

#include "simscene.h"

#include <RPIMoCap/ClientLib/icamera.h>

#include <QElapsedTimer>
#include <Eigen/Geometry>

namespace RPIMoCap::SimClient {

class SimCamera : public ICamera
{
public:
    SimCamera(const Camera::Intrinsics &params, const SimScene &scene);
    ~SimCamera() = default;

    SimCamera(const SimCamera&) = delete;
    void operator=(const SimCamera&) = delete;

    bool getOpened() const override {return m_opened;}
    bool open() override;
    void close() override;

    cv::Mat pullData() override;

    Camera::Intrinsics& getParams();

    Eigen::Vector3d getTranslation() const;
    void setTranslation(const Eigen::Vector3d &translation);

    Eigen::Vector3d getRotation() const;
    void setRotation(const Eigen::Vector3d &rotation);

private:
    bool m_opened = false;

    Camera::Intrinsics m_params;
    const SimScene &m_scene;

    QElapsedTimer m_timer;

    Eigen::Vector3d m_translation = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_rotation = Eigen::Vector3d::Zero();
};

}
