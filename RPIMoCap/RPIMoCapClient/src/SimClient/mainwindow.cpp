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

#include "RPIMoCap/SimClient/mainwindow.h"
#include "ui_mainwindow.h"


#include <RPIMoCap/SimClient/virtualwand.h>
#include <RPIMoCap/SimClient/simcamerawidget.h>

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

    ui->scrollAreaWidgetContents->setLayout(new QVBoxLayout);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::updateValue()
{
    VirtualWand wand(100.0, 20);

    Eigen::Vector3f t(ui->valuex->value(),
                      ui->valuey->value(),
                      ui->valuez->value());

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translate(t);

    std::vector<SimMarker> markers = wand.markers(transform);

    m_scene.setMarkers(markers);
}

void MainWindow::on_addClientButton_clicked()
{
    RPIMoCap::CameraParams params = RPIMoCap::CameraParams::computeRPICameraV1Params();

    auto camera = std::make_shared<SimCamera>(params, m_scene);
    auto client = QSharedPointer<RPIMoCapClient>::create(camera,params);
    auto widget = new SimCameraWidget(camera);
    ui->scrollAreaWidgetContents->layout()->addWidget(widget);

    m_clients.push_back(client);
    m_clientWidgets.push_back(widget);
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
}

}
