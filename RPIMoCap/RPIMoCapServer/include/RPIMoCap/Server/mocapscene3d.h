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

#include <RPIMoCap/Server/camerasettings.h>
#include <RPIMoCap/Core/frame.h>

#include <QWidget>

#include <Qt3DCore>
#include <Qt3DExtras>
#include <Qt3DRender/QGeometryRenderer>

namespace Ui {
class MocapScene3D;
}

class MocapScene3D : public QWidget
{
    Q_OBJECT

public:
    explicit MocapScene3D(QWidget *parent = nullptr);
    ~MocapScene3D();

    void addCamera(const std::shared_ptr<CameraSettings> &camera);
    void removeCamera(const int id);

    void drawFrame(const RPIMoCap::Frame &frame);

private slots:
    void updateCameras();

private:
    class Camera
    {
    public:
        Camera(Eigen::Affine3f transform,Qt3DCore::QEntity *rootEntity) {
            setTransform(transform);

            cameraMesh->setRings(4);
            cameraMesh->setSlices(4);
            cameraMesh->setLength(10);
            cameraMesh->setTopRadius(6.0f);
            cameraMesh->setBottomRadius(0.1f);
            cameraMesh->setHasBottomEndcap(true);
            cameraMesh->setHasTopEndcap(true);

            cameraMaterial->setDiffuse(QColor(QRgb(0x575757)));

            entity->addComponent(cameraTransform);
            entity->addComponent(cameraMesh);
            entity->addComponent(cameraMaterial);
            entity->setParent(rootEntity);
        }

        ~Camera()
        {
            entity->deleteLater();
        }

        void setTransform(Eigen::Affine3f transform)
        {
            Eigen::Affine3f coneTransform = Eigen::Affine3f(Eigen::Affine3f::Identity());
            coneTransform.rotate(Eigen::AngleAxisf(0.5*M_PI, Eigen::Vector3f::UnitX()));
            coneTransform.rotate(Eigen::AngleAxisf(0.25*M_PI, Eigen::Vector3f::UnitY()));
            transform = transform * coneTransform;

            cameraTransform->setTranslation({transform.translation().x(),
                                             transform.translation().y(),
                                             transform.translation().z()});

            const Eigen::Quaternionf rot(transform.rotation());
            cameraTransform->setRotation(QQuaternion(QVector4D(rot.x(),rot.y(),rot.z(),rot.w())));
        }

        Qt3DCore::QEntity *entity = new Qt3DCore::QEntity();
        Qt3DCore::QTransform *cameraTransform = new Qt3DCore::QTransform();
        Qt3DExtras::QConeMesh *cameraMesh = new Qt3DExtras::QConeMesh();
        Qt3DExtras::QPhongMaterial *cameraMaterial = new Qt3DExtras::QPhongMaterial();
    };

    struct Marker
    {
    public:
        Marker(const RPIMoCap::Frame::Marker &marker,Qt3DCore::QEntity *rootEntity) {
            sphereTransform->setTranslation({marker.position.x(),marker.position.y(),marker.position.z()});

            sphereMesh->setRings(20);
            sphereMesh->setSlices(20);
            sphereMesh->setRadius(2);

            sphereMaterial->setDiffuse(QColor(QRgb(0x0026ff)));

            entity->addComponent(sphereTransform);
            entity->addComponent(sphereMesh);
            entity->addComponent(sphereMaterial);
            entity->setParent(rootEntity);
        }

        ~Marker() {
            entity->deleteLater();
        }

        Qt3DCore::QEntity *entity = new Qt3DCore::QEntity();
        Qt3DCore::QTransform *sphereTransform = new Qt3DCore::QTransform();
        Qt3DExtras::QSphereMesh *sphereMesh = new Qt3DExtras::QSphereMesh();
        Qt3DExtras::QPhongMaterial *sphereMaterial = new Qt3DExtras::QPhongMaterial();
    };

    struct Line
    {
        Line(const RPIMoCap::Line3D &line, Qt3DCore::QEntity *rootEntity)
        {
            buf = new Qt3DRender::QBuffer(geometry);
            setLine3D(line);

            auto *positionAttribute = new Qt3DRender::QAttribute(geometry);
            positionAttribute->setName(Qt3DRender::QAttribute::defaultPositionAttributeName());
            positionAttribute->setVertexBaseType(Qt3DRender::QAttribute::Float);
            positionAttribute->setVertexSize(3);
            positionAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
            positionAttribute->setBuffer(buf);
            positionAttribute->setByteStride(3 * sizeof(float));
            positionAttribute->setCount(2);
            geometry->addAttribute(positionAttribute); // We add the vertices in the geometry

            // connectivity between vertices
            QByteArray indexBytes;
            indexBytes.resize(2 * sizeof(unsigned int)); // start to end
            unsigned int *indices = reinterpret_cast<unsigned int*>(indexBytes.data());
            *indices++ = 0;
            *indices++ = 1;

            auto *indexBuffer = new Qt3DRender::QBuffer(geometry);
            indexBuffer->setData(indexBytes);

            auto *indexAttribute = new Qt3DRender::QAttribute(geometry);
            indexAttribute->setVertexBaseType(Qt3DRender::QAttribute::UnsignedInt);
            indexAttribute->setAttributeType(Qt3DRender::QAttribute::IndexAttribute);
            indexAttribute->setBuffer(indexBuffer);
            indexAttribute->setCount(2);
            geometry->addAttribute(indexAttribute); // We add the indices linking the points in the geometryS

            // mesh
            renderer->setGeometry(geometry);
            renderer->setPrimitiveType(Qt3DRender::QGeometryRenderer::Lines);
            lineMaterial->setAmbient(QColor(QRgb(0x26e61c)));

            entity->addComponent(renderer);
            entity->addComponent(lineMaterial);
            entity->setParent(rootEntity);
        }

        ~Line()
        {
            entity->deleteLater();
        }

        void setLine3D(const RPIMoCap::Line3D &line)
        {
            bufferBytes.resize(3 * 2 * sizeof(float)); // start.x, start.y, start.end + end.x, end.y, end.z
            float *positions = reinterpret_cast<float*>(bufferBytes.data());
            *positions++ = line.origin().x();
            *positions++ = line.origin().y();
            *positions++ = line.origin().z();

            const auto end = line.origin() + 400.0 * line.direction(); //TODO add length
            *positions++ = end.x();
            *positions++ = end.y();
            *positions++ = end.z();

            buf->setData(bufferBytes);
        }

        Qt3DCore::QEntity *entity = new Qt3DCore::QEntity();
        Qt3DRender::QGeometry *geometry = new Qt3DRender::QGeometry();
        Qt3DExtras::QPhongMaterial *lineMaterial = new Qt3DExtras::QPhongMaterial();
        Qt3DRender::QGeometryRenderer *renderer = new Qt3DRender::QGeometryRenderer();
        Qt3DRender::QBuffer *buf;
        QByteArray bufferBytes;

    };

    Ui::MocapScene3D *ui;

    Qt3DCore::QEntity *m_rootEntity = new Qt3DCore::QEntity();

    std::vector<Marker> m_currentMarkers;
    std::vector<Line*> m_allLines;

    QHash<int,std::pair<std::shared_ptr<CameraSettings>,std::shared_ptr<Camera>>> m_currentCameras;
};
