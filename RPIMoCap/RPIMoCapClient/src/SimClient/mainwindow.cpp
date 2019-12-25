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

namespace RPIMoCap::SimClient {

MainWindow::MainWindow(SimScene &scene, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_scene(scene)
{
    ui->setupUi(this);

    connect(ui->valuex,qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateValue);
    connect(ui->valuey,qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateValue);
    connect(ui->valuez,qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateValue);
    connect(ui->rotX,qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateValue);
    connect(ui->rotY,qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateValue);
    connect(ui->rotZ,qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateValue);

    ui->scrollAreaWidgetContents->setLayout(new QVBoxLayout);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::updateValue()
{
    const VirtualWand wand(50.0, 10.0);

    const Eigen::Vector3f pos(ui->valuex->value(),
                              ui->valuey->value(),
                              ui->valuez->value());

    const Eigen::Matrix3f r(Eigen::AngleAxisf(ui->rotX->value()*M_PI/180.0f, Eigen::Vector3f::UnitX())
                      * Eigen::AngleAxisf(ui->rotY->value()*M_PI/180.0f,  Eigen::Vector3f::UnitY())
                      * Eigen::AngleAxisf(ui->rotZ->value()*M_PI/180.0f, Eigen::Vector3f::UnitZ()));

    const Eigen::Vector3f s(1.0, 1.0, 1.0);

    Eigen::Affine3f t;
    t.fromPositionOrientationScale(pos, r, s);
    m_scene.setMarkers(wand.markers(t));
}

void MainWindow::on_addClientButton_clicked()
{
    RPIMoCap::CameraParams params = RPIMoCap::CameraParams::computeRPICameraV1Params();

    auto camera = std::make_shared<SimCamera>(params, m_scene);
    auto client = QSharedPointer<Client>::create(camera,params);
    auto widget = new SimCameraWidget(camera);
    auto thread = new QThread;
    ui->scrollAreaWidgetContents->layout()->addWidget(widget);
    client->moveToThread(thread);
    thread->start();

    m_clients.push_back(client);
    m_clientWidgets.push_back(widget);
    m_clientThreads.push_back(thread);
}

void MainWindow::on_removeClientButton_clicked()
{
    if (m_clients.empty())
    {
        return;
    }

    QWidget *last = m_clientWidgets.last();
    ui->scrollAreaWidgetContents->layout()->removeWidget(last);
    last->deleteLater();

    m_clientWidgets.removeLast();
    m_clients.removeLast();

    QThread *lastThread = m_clientThreads.last();
    lastThread->terminate();
    lastThread->deleteLater();
    m_clientThreads.removeLast();
}

}
