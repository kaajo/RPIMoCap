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

#include "RPIMoCap/mocapscene3d.h"
#include "ui_mocapscene3d.h"

#include <Qt3DRender/QCamera>
#include <Qt3DRender/QPointLight>

#include <Qt3DInput/QInputAspect>

#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DExtras/QFirstPersonCameraController>
#include <Qt3DExtras/QPlaneMesh>
#include <Qt3DExtras>

namespace RPIMoCap::Visualization {

MocapScene3D::MocapScene3D(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MocapScene3D)
{
    ui->setupUi(this);

    Qt3DExtras::Qt3DWindow *view = new Qt3DExtras::Qt3DWindow();
    Qt3DInput::QInputAspect *input = new Qt3DInput::QInputAspect;
    view->registerAspect(input);
    view->renderSettings()->setRenderPolicy(Qt3DRender::QRenderSettings::OnDemand);

    ui->verticalLayout->addWidget(QWidget::createWindowContainer(view));

    Qt3DRender::QCamera *cameraEntity = view->camera();
    cameraEntity->lens()->setPerspectiveProjection(90.0f, 16.0f/9.0f, 0.1f, 2000.0f);
    cameraEntity->setPosition(QVector3D(400, 400, 400));
    cameraEntity->setUpVector(QVector3D(0, 1, 0));
    cameraEntity->setViewCenter(QVector3D(0, 0, 0));

    Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(m_rootEntity);
    Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity);
    light->setColor("white");
    light->setIntensity(1);
    lightEntity->addComponent(light);
    Qt3DCore::QTransform *lightTransform = new Qt3DCore::QTransform(lightEntity);
    lightTransform->setTranslation(cameraEntity->position());
    lightEntity->addComponent(lightTransform);
    Qt3DExtras::QOrbitCameraController *camController = new Qt3DExtras::QOrbitCameraController(m_rootEntity);
    camController->setZoomInLimit(20.0f);
    camController->setCamera(cameraEntity);
    camController->setLinearSpeed(1500);

    for (size_t i = 0; i < 200; ++i)
    {
        auto line = new Line(Line3D(), 100, m_rootEntity);
        m_allLines.push_back(line);
        line->entity->setEnabled(false);
    }

    for (size_t i = 0; i < 200; ++i)
    {
        auto marker = new Marker({0, {}, Eigen::Vector3f()}, m_rootEntity);
        m_currentMarkers.push_back(marker);
        marker->entity->setEnabled(false);
    }

    m_floor = new FloorPlane(m_rootEntity);

    view->setRootEntity(m_rootEntity);
}

MocapScene3D::~MocapScene3D()
{
    delete ui;
}

void MocapScene3D::addCamera(const QUuid id, const Eigen::Affine3f &transform)
{
    m_currentCameras.insert(id, std::make_shared<Camera>(transform, m_rootEntity));
}

void MocapScene3D::updateCamera(const QUuid id, const Eigen::Affine3f &transform)
{
    m_currentCameras.value(id)->setTransform(transform);
}

void MocapScene3D::removeCamera(const QUuid id)
{
    m_currentCameras.remove(id);
}

void MocapScene3D::drawFrame(const RPIMoCap::Frame &frame)
{
    //TODO resize m_allLines and m_currentMarkers if needed

    size_t lineCounter = 0;

    for (const auto&obs : frame.observations()) {
        for (size_t i = 0; i < obs.second.size(); ++i)
        {
            m_allLines[lineCounter]->setLine3D(obs.second[i].ray, 300); // TODO get length from marker
            m_allLines[lineCounter]->entity->setEnabled(true);
            ++lineCounter;
        }
    }

    for (size_t i = lineCounter + 1; i < m_allLines.size(); ++i)
    {
        m_allLines[i]->entity->setEnabled(false);
    }

    for (size_t i = 0; i < frame.markers().size(); ++i)
    {
        m_currentMarkers[i]->setMarker(frame.markers()[i]);
        m_currentMarkers[i]->entity->setEnabled(true);
    }

    for (size_t i = frame.markers().size(); i < m_currentMarkers.size(); ++i)
    {
        m_currentMarkers[i]->entity->setEnabled(false);
    }
}

}
