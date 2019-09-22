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

#include "RPIMoCap/Server/mocapscene3d.h"
#include "ui_mocapscene3d.h"

#include <Qt3DRender/QCamera>
#include <Qt3DRender/QPointLight>

#include <Qt3DInput/QInputAspect>

#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DExtras/QFirstPersonCameraController>
#include <Qt3DExtras>

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
    cameraEntity->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    cameraEntity->setPosition(QVector3D(100, 0, 100));
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
    camController->setCamera(cameraEntity);

    for (size_t i = 0; i < 100; ++i)
    {
        m_allLines.push_back(new Line(RPIMoCap::Line3D(),m_rootEntity));
    }

    view->setRootEntity(m_rootEntity);
}

MocapScene3D::~MocapScene3D()
{
    delete ui;
}

void MocapScene3D::addCamera(const std::shared_ptr<CameraSettings> &camera)
{
    m_currentCameras[camera->id()] = {camera,std::make_shared<Camera>(camera->transform(), m_rootEntity)};
    connect(camera.get(), &CameraSettings::changed, this, &MocapScene3D::updateCameras);
}

void MocapScene3D::removeCamera(const int id)
{
    m_currentCameras.remove(id);
}

void MocapScene3D::drawFrame(const RPIMoCap::Frame &frame)
{
    for (size_t i = 0; i < frame.lines().size(); ++i)
    {
        m_allLines[i]->setLine3D(frame.lines()[i]);
        m_allLines[i]->entity->setEnabled(true);
    }

    for (size_t i = frame.lines().size(); i < m_allLines.size(); ++i)
    {
        m_allLines[i]->entity->setEnabled(false);
    }

    update(); //TODO needed?
}

void MocapScene3D::updateCameras()
{
    for (auto &cam : m_currentCameras)
    {
        cam.second->setTransform(cam.first->transform());
    }
}
