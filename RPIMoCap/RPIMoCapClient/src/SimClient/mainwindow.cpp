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

#include "ui_mainwindow.h"

#include "RPIMoCap/SimClient/mainwindow.h"
#include "RPIMoCap/SimClient/virtualwand.h"
#include "RPIMoCap/SimClient/simcamerawidget.h"

#include <QThread>
#include <QCloseEvent>

namespace RPIMoCap::SimClient {

MainWindow::MainWindow(SimScene &scene, QWidget *parent) :
    QMainWindow(parent),
    m_ui(new Ui::MainWindow),
    m_scene(scene)
{
    m_ui->setupUi(this);

    connect(m_ui->wandExtrinsic, &Visualization::ExtrinsicWidget::transformChanged, this, &MainWindow::updateWandTransform);

    m_ui->scrollAreaWidgetContents->setLayout(new QVBoxLayout);
}

MainWindow::~MainWindow()
{
    delete m_ui;
}

void MainWindow::updateWandTransform(Eigen::Affine3f transform)
{
    const VirtualWand wand(50.0, 10.0);

    auto markers = wand.markers(transform);
    m_scene.setMarkers(markers);

    std::vector<Frame::Marker> frameMarkers;

    for (auto &marker : markers)
    {
        frameMarkers.push_back({0,{marker.translation.x, marker.translation.y, marker.translation.z}});
    }

    Frame frame(std::chrono::high_resolution_clock::now(), {});
    frame.setMarkers(frameMarkers);

    m_ui->scene->drawFrame(frame);
}

void MainWindow::onRotationChanged(QUuid clientId, cv::Vec3f rVec)
{
    auto it = std::find_if(m_clients.begin(), m_clients.end(),
                           [&clientId](ClientData &c){return c.id == clientId;});

    if (it != m_clients.end())
    {
        it->camera->setRotation(rVec);
    }
    else
    {
        qWarning() << clientId << " client does not exist!";
    }
}

void MainWindow::onTranslationChanged(QUuid clientId, cv::Vec3f tVec)
{
    auto it = std::find_if(m_clients.begin(), m_clients.end(),
                           [&clientId](ClientData &c){return c.id == clientId;});

    if (it != m_clients.end())
    {
        it->camera->setTranslation(tVec);
    }
    else
    {
        qWarning() << clientId << " client does not exist!";
    }
}

void MainWindow::onfpsChanged(QUuid clientId, int64_t fps)
{
    auto it = std::find_if(m_clients.begin(), m_clients.end(),
                           [&clientId](ClientData &c){return c.id == clientId;});

    if (it != m_clients.end())
    {
        it->camera->getParams().maxFPS = fps;
    }
    else
    {
        qWarning() << clientId << " client does not exist!";
    }
}

void MainWindow::on_addClientButton_clicked()
{
    RPIMoCap::Camera::Intrinsics params = RPIMoCap::Camera::Intrinsics::computeRPICameraV1Params();

    auto camera = std::make_shared<SimCamera>(params, m_scene);
    auto client = QSharedPointer<Client>(new Client(camera,params), &QObject::deleteLater);
    auto widget = new SimCameraWidget(camera->getParams().maxFPS, client->id());

    connect(widget, &SimCameraWidget::fpsChanged, this, &MainWindow::onfpsChanged);
    connect(widget, &SimCameraWidget::transformChanged, m_ui->scene, &Visualization::MocapScene3D::updateCamera);
    connect(widget, &SimCameraWidget::rotationChanged, this, &MainWindow::onRotationChanged);
    connect(widget, &SimCameraWidget::translationChanged, this, &MainWindow::onTranslationChanged);

    m_ui->scene->addCamera(client->id(), Eigen::Affine3f::Identity());

    auto thread = new QThread;
    m_ui->scrollAreaWidgetContents->layout()->addWidget(widget);
    client->moveToThread(thread);
    thread->start();

    m_clients.push_back({client->id(), camera, client, widget, thread});
}

void MainWindow::on_removeClientButton_clicked()
{
    if (m_clients.empty())
    {
        return;
    }

    auto &last = m_clients.last();
    m_ui->scene->removeCamera(last.id);
    m_ui->scrollAreaWidgetContents->layout()->removeWidget(last.widget);

    last.clear();

    m_clients.removeLast();
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    event->accept();

    for (auto &client : m_clients)
    {
        client.clear(); //TODO oh my
    }

    m_clients.clear();
}

}
