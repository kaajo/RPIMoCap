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

#include "RPIMoCap/Server/calibrationwidget.h"

#include <QTreeWidgetItem>

namespace RPIMoCap {

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_calibWidget(new CalibrationWidget())
{
    ui->setupUi(this);

    ui->scrollAreaWidgetContents->setLayout(new QVBoxLayout);
    ui->scrollAreaWidgetContents->layout()->setSpacing(0);
    ui->scrollAreaWidgetContents->layout()->setMargin(0);

    connect(ui->searchCameras, &QPushButton::clicked,
            this, &MainWindow::searchForCameras);

    { // Add calibration section
        QTreeWidgetItem* pCategory = new QTreeWidgetItem();
        pCategory->setText(0, "Calibration");
        pCategory->setTextAlignment(0, Qt::AlignCenter);
        ui->treeWidget->addTopLevelItem(pCategory);

        QTreeWidgetItem* pContainer = new QTreeWidgetItem();
        pCategory->addChild(pContainer);
        ui->treeWidget->setItemWidget(pContainer, 0, m_calibWidget);

        pCategory->setExpanded(true);
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::addCamera(const std::shared_ptr<CameraSettings> &camera)
{
    connect(camera.get(), &CameraSettings::rotationChanged, this, &MainWindow::updateCamera);
    connect(camera.get(), &CameraSettings::translationChanged, this, &MainWindow::updateCamera);

    ui->scene->addCamera(camera->id(), camera->transform());

    auto widget = new CameraSettingsWidget(camera->id());

    connect(camera.get(), &CameraSettings::rotationChanged,
            widget->extrinsic(), &Visualization::ExtrinsicWidget::setRotation);
    connect(camera.get(), &CameraSettings::translationChanged,
            widget->extrinsic(), &Visualization::ExtrinsicWidget::setTranslation);

    connect(widget->extrinsic(), &Visualization::ExtrinsicWidget::rotationChanged,
            camera.get(), &CameraSettings::setRotation);
    connect(widget->extrinsic(), &Visualization::ExtrinsicWidget::translationChanged,
            camera.get(), &CameraSettings::setTranslation);

    m_cameraWidgets[camera->id()] = widget;
    ui->scrollAreaWidgetContents->layout()->addWidget(widget);
}

void MainWindow::updateCamera()
{
    CameraSettings* snd = qobject_cast<CameraSettings*>(sender());
    ui->scene->updateCamera(snd->id(), snd->transform());
}

void MainWindow::removeCamera(const QUuid id)
{
    ui->scene->removeCamera(id);

    ui->scrollAreaWidgetContents->layout()->removeWidget(m_cameraWidgets[id]);
    m_cameraWidgets[id]->deleteLater();
    m_cameraWidgets.remove(id);
}

void MainWindow::drawFrame(const Frame &frame)
{
    ui->scene->drawFrame(frame);
}

void MainWindow::on_MoCapButton_clicked(bool checked)
{
    ui->MoCapButton->setText(checked ? "STOP" : "START");
    emit startMoCap(checked);
}

}
