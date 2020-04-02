/*
 * This file is part of the RPIMoCap (https://github.com/kaajo/RPIMoCap).
 * Copyright (c) 2020 Miroslav Krajicek.
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

#include <RPIMoCap/Core/frame.h>
#include <RPIMoCap/Core/line3d.h>

#include <Qt3DCore>
#include <Qt3DExtras>

#include <Eigen/Geometry>

namespace RPIMoCap::Visualization {

using SharedEntity = QSharedPointer<Qt3DCore::QEntity>;

class Camera
{
public:
    Camera(Eigen::Affine3f transform, Qt3DCore::QEntity *rootEntity);

    void setTransform(Eigen::Affine3f transform);

    SharedEntity entity = SharedEntity(new Qt3DCore::QEntity(), &QObject::deleteLater);
    Qt3DCore::QTransform *cameraTransform = new Qt3DCore::QTransform();
    Qt3DExtras::QConeMesh *cameraMesh = new Qt3DExtras::QConeMesh();
    Qt3DExtras::QPhongMaterial *cameraMaterial = new Qt3DExtras::QPhongMaterial();
};

struct Marker
{
public:
    Marker(const RPIMoCap::Frame::Marker marker, Qt3DCore::QEntity *rootEntity);

    void setMarker(const RPIMoCap::Frame::Marker marker);

    SharedEntity entity = SharedEntity(new Qt3DCore::QEntity(), &QObject::deleteLater);
    Qt3DCore::QTransform *sphereTransform = new Qt3DCore::QTransform();
    Qt3DExtras::QSphereMesh *sphereMesh = new Qt3DExtras::QSphereMesh();
    Qt3DExtras::QPhongMaterial *sphereMaterial = new Qt3DExtras::QPhongMaterial();
};

struct Line
{
    Line(const RPIMoCap::Frame::LineSegment &line, Qt3DCore::QEntity *rootEntity);

    void setLine3D(const RPIMoCap::Frame::LineSegment &line);

    SharedEntity entity = SharedEntity(new Qt3DCore::QEntity(), &QObject::deleteLater);
    Qt3DRender::QGeometry *geometry = new Qt3DRender::QGeometry();
    Qt3DExtras::QPhongMaterial *lineMaterial = new Qt3DExtras::QPhongMaterial();
    Qt3DRender::QGeometryRenderer *renderer = new Qt3DRender::QGeometryRenderer();
    Qt3DRender::QBuffer *buf;
    QByteArray bufferBytes;
};

}
