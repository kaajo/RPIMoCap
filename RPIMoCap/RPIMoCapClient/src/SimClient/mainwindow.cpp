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

#include <RPIMoCap/Core/topics.h>

#include <QThread>
#include <QCloseEvent>
#include <QFileDialog>
#include <QJsonDocument>

namespace RPIMoCap::SimClient {

MainWindow::MainWindow(SimScene &scene, QWidget *parent) :
    QMainWindow(parent),
    m_ui(new Ui::MainWindow),
    m_scene(scene)
{
    m_ui->setupUi(this);

    connect(m_ui->wandExtrinsic, &Visualization::ExtrinsicWidget::transformChanged, this, &MainWindow::drawFrame);
    connect(m_ui->floorCross, &Visualization::ExtrinsicWidget::transformChanged, this, &MainWindow::drawFrame);
    connect(m_ui->showWandExtrinsic, &QCheckBox::stateChanged, this, &MainWindow::drawFrame);
    connect(m_ui->showFloorCross, &QCheckBox::stateChanged, this, &MainWindow::drawFrame);

    m_ui->scrollAreaWidgetContents->setLayout(new QVBoxLayout);

    connect(m_ui->actionOpen, &QAction::triggered, this, &MainWindow::openProject);
    connect(m_ui->actionSave, &QAction::triggered, this, &MainWindow::saveProject);
    connect(m_ui->addClientButton, &QPushButton::clicked, this, &MainWindow::createClient);
    connect(m_ui->removeClientButton, &QPushButton::clicked, this, &MainWindow::removeClient);
}

MainWindow::~MainWindow()
{
    delete m_ui;
}

void MainWindow::onRotationChanged(QUuid clientId, Eigen::Vector3d rVec)
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

void MainWindow::onTranslationChanged(QUuid clientId, Eigen::Vector3d tVec)
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

void MainWindow::openProject()
{
    QSettings settings;
    settings.beginGroup("simulation");
    const QString lastDir = settings.value("lastProject", QStandardPaths::DocumentsLocation).toString();
    const QString path = QFileDialog::getOpenFileName(this, "Save project", lastDir);

    if (path.isNull() || path.isEmpty())
    {
        return;
    }

    clearClients();

    QFile loadFile(path);
    loadFile.open(QIODevice::ReadOnly);

    const QVariantMap data = QJsonDocument::fromJson(loadFile.readAll()).toVariant().toMap();
    QVariantList clients = data["clients"].toList();

    for (auto &var : clients)
    {
        auto map = var.toMap();

        QUuid id = QUuid::fromString(map["id"].toString());
        auto params = Camera::Intrinsics::fromVariantMap(map);

        Eigen::Vector3d rVec(map["rx"].toDouble(), map["ry"].toDouble(), map["rz"].toDouble());
        Eigen::Vector3d tVec(map["tx"].toDouble(), map["ty"].toDouble(), map["tz"].toDouble());
        addClient(id, params, rVec, tVec);
    }

    settings.setValue("lastProject", QFileInfo(path).dir().path());
}

void MainWindow::saveProject()
{
    QSettings settings;
    settings.beginGroup("simulation");
    const QString lastDir = settings.value("lastProject", QStandardPaths::DocumentsLocation).toString();
    const QString path = QFileDialog::getSaveFileName(this, "Save project", lastDir);

    if (path.isNull() || path.isEmpty())
    {
        return;
    }

    QVariantMap saveMap;
    QVariantList clientList;
    for (const auto& data : m_clients)
    {
        QVariantMap clientMap = data.camera->getParams().toVariantMap();
        clientMap["id"] = data.id.toString();

        {
            auto rVec = data.camera->getRotation();
            clientMap["rx"] = rVec[0];
            clientMap["ry"] = rVec[1];
            clientMap["rz"] = rVec[2];
        }
        {
            auto tVec = data.camera->getTranslation();
            clientMap["tx"] = tVec[0];
            clientMap["ty"] = tVec[1];
            clientMap["tz"] = tVec[2];
        }

        clientList.push_back(clientMap);
    }
    saveMap["clients"] = clientList;

    auto data = QJsonDocument::fromVariant(saveMap).toJson(QJsonDocument::JsonFormat::Indented);
    QFile saveFile(path);
    saveFile.open(QIODevice::WriteOnly);
    saveFile.write(data);
    saveFile.close();

    settings.setValue("lastProject", QFileInfo(path).dir().path());
}

void MainWindow::createClient()
{
    RPIMoCap::Camera::Intrinsics params = RPIMoCap::Camera::Intrinsics::computeRPICameraV1Params();
    QUuid id = QUuid::createUuid();
    addClient(id, params, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
}

void MainWindow::removeClient()
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
    clearClients();
}

void MainWindow::addClient(const QUuid &id, const Camera::Intrinsics &params,
                           const Eigen::Vector3d &rVec, const Eigen::Vector3d &tVec)
{
    auto camera = std::make_shared<SimCamera>(params, m_scene);

    auto client = QSharedPointer<Client>(new Client(camera,params, id), &QObject::deleteLater);
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

    camera->setRotation(rVec);
    camera->setTranslation(tVec);
    widget->extrinsic()->setRotation(rVec);
    widget->extrinsic()->setTranslation(tVec);
}

void MainWindow::clearClients()
{
    while (!m_clients.empty())
    {
        removeClient();
    }
}

void MainWindow::drawFrame()
{
    std::vector<SimMarker> markers;

    if (m_ui->showWandExtrinsic->isChecked()) {
        const VirtualWand wand(50.0, 10.0);
        auto wandMarkers = wand.markers(m_ui->wandExtrinsic->getTransform());
        markers.insert(markers.end(), wandMarkers.begin(), wandMarkers.end());
    }

    if (m_ui->showFloorCross->isChecked()) {
        const VirtualFloorWand wand(30.0);
        auto wandMarkers = wand.markers(m_ui->floorCross->getTransform());
        markers.insert(markers.end(), wandMarkers.begin(), wandMarkers.end());
    }

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

}
