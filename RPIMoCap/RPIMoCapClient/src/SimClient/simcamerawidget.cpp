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

#include "RPIMoCap/SimClient/simcamerawidget.h"
#include "ui_simcamerawidget.h"

namespace RPIMoCap::SimClient {

constexpr double degToRad = M_PI/180.0;
constexpr double radToDeg = 180.0/M_PI;

SimCameraWidget::SimCameraWidget(uint16_t fps, QUuid clientId, QWidget *parent) :
    QWidget(parent),
    m_ui(new Ui::SimCameraWidget),
    m_clientId(clientId)
{
    m_ui->setupUi(this);

    m_ui->cameraidvalue->setText(clientId.toString(QUuid::StringFormat::WithoutBraces).left(16));
    m_ui->fpsvalue->setValue(fps);

    connect(m_ui->fpsvalue,  QOverload<int>::of(&QSpinBox::valueChanged), this, &SimCameraWidget::onfpsChanged);
    connect(m_ui->extrinsic, &Visualization::ExtrinsicWidget::rotationChanged, this, &SimCameraWidget::onRotationChanged);
    connect(m_ui->extrinsic, &Visualization::ExtrinsicWidget::translationChanged, this, &SimCameraWidget::onTranslationChanged);
    connect(m_ui->extrinsic, &Visualization::ExtrinsicWidget::transformChanged, this, &SimCameraWidget::onTransformChanged);
}

SimCameraWidget::~SimCameraWidget()
{
    delete m_ui;
}

Visualization::ExtrinsicWidget *SimCameraWidget::extrinsic()
{
    return m_ui->extrinsic;
}

void SimCameraWidget::onfpsChanged(int fps)
{
    emit fpsChanged(m_clientId, fps);
}

void SimCameraWidget::onRotationChanged(Eigen::Vector3d rVec)
{
    emit rotationChanged(m_clientId, rVec);
}

void SimCameraWidget::onTranslationChanged(Eigen::Vector3d tVec)
{
    emit translationChanged(m_clientId, tVec);
}

void SimCameraWidget::onTransformChanged(Eigen::Affine3f transform)
{
    emit transformChanged(m_clientId, transform);
}

}
