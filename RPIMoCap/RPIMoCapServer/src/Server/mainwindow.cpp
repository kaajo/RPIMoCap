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

#include "RPIMoCap/Server/mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->scrollAreaWidgetContents->setLayout(new QVBoxLayout);
    ui->scrollAreaWidgetContents->layout()->setSpacing(0);
    ui->scrollAreaWidgetContents->layout()->setMargin(0);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onLinesReceived(const std::vector<RPIMoCap::Line3D> &lines)
{
    RPIMoCap::Frame frame(0,lines);;
    ui->widget->drawFrame(frame);
}

void MainWindow::addCamera(const std::shared_ptr<CameraSettings> &camera)
{
    ui->widget->addCamera(camera);

    auto widget = new CameraSettingsWidget(camera);
    m_cameraWidgets[camera->id()] = widget;
    ui->scrollAreaWidgetContents->layout()->addWidget(widget);
}

void MainWindow::removeCamera(const int id)
{
    ui->widget->removeCamera(id);

    ui->scrollAreaWidgetContents->layout()->removeWidget(m_cameraWidgets[id]);
    m_cameraWidgets[id]->deleteLater();
    m_cameraWidgets.remove(id);
}

void MainWindow::on_MoCapButton_clicked(bool checked)
{
    ui->MoCapButton->setText(checked ? "STOP" : "START");
    emit startMoCap(checked);
}

void MainWindow::on_calibButton_clicked(bool checked)
{
    ui->calibButton->setText(checked ? "STOP calib" : "START calib");
    emit startCalib(checked);
}
