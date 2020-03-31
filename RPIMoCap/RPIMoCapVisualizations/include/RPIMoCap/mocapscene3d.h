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

#include "RPIMoCap/primitives.h"

#include <RPIMoCap/Core/frame.h>

#include <QWidget>
#include <Qt3DRender/QGeometryRenderer>

namespace Ui {
class MocapScene3D;
}

namespace RPIMoCap::Visualization {

class MocapScene3D : public QWidget
{
    Q_OBJECT

public:
    explicit MocapScene3D(QWidget *parent = nullptr);
    ~MocapScene3D();

public slots:
    void addCamera(const QUuid id, const Eigen::Affine3f &transform);
    void updateCamera(const QUuid id, const Eigen::Affine3f &transform);
    void removeCamera(const QUuid id);

    void drawFrame(const RPIMoCap::Frame &frame);

private:
    Ui::MocapScene3D *ui = nullptr;

    Qt3DCore::QEntity *m_rootEntity = new Qt3DCore::QEntity();

    std::vector<Marker> m_currentMarkers;
    std::vector<Line*> m_allLines;

    QHash<QUuid,std::shared_ptr<Camera>> m_currentCameras;
};

}
