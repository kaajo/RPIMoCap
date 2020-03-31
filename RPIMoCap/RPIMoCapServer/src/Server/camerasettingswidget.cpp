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

#include "RPIMoCap/Server/camerasettingswidget.h"
#include "ui_camerasettingswidget.h"

namespace RPIMoCap {

CameraSettingsWidget::CameraSettingsWidget(const std::shared_ptr<CameraSettings> camera, QWidget *parent)
    : QWidget(parent)
    , m_camera(camera)
    , ui(new Ui::CameraSettingsWidget)
{
    ui->setupUi(this);
    ui->cameraid->setText(camera->id().toString(QUuid::StringFormat::WithoutBraces));
    connect(m_camera.get(), &CameraSettings::rotationChanged, this, &CameraSettingsWidget::setRotation);
    connect(m_camera.get(), &CameraSettings::translationChanged, this, &CameraSettingsWidget::setTranslation);

    setRotation(m_camera->property("rotation").value<cv::Vec3f>());
    setTranslation(m_camera->property("translation").value<cv::Vec3f>());
}

CameraSettingsWidget::~CameraSettingsWidget()
{
    delete ui;
}

void CameraSettingsWidget::on_pushButton_clicked()
{
    const float rx = ui->rotationX->value() * M_PI/180.0;
    const float ry = ui->rotationY->value() * M_PI/180.0;
    const float rz = ui->rotationZ->value() * M_PI/180.0;
    m_camera->setProperty("rotation", QVariant::fromValue(cv::Vec3f(rx, ry, rz)));

    const float tx = ui->translationX->value();
    const float ty = ui->translationY->value();
    const float tz = ui->translationZ->value();
    m_camera->setProperty("translation", QVariant::fromValue(cv::Vec3f(tx, ty, tz)));
}

void CameraSettingsWidget::setRotation(const cv::Vec3f &rVec)
{
    ui->rotationX->setValue(rVec[0] * 180.0/M_PI);
    ui->rotationY->setValue(rVec[1] * 180.0/M_PI);
    ui->rotationZ->setValue(rVec[2] * 180.0/M_PI);
}

void CameraSettingsWidget::setTranslation(const cv::Vec3f &tVec)
{
    ui->translationX->setValue(tVec[0]);
    ui->translationY->setValue(tVec[1]);
    ui->translationZ->setValue(tVec[2]);
}

}
