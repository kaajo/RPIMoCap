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

SimCameraWidget::SimCameraWidget(std::shared_ptr<SimCamera> camera, QWidget *parent) :
    QWidget(parent),
    m_ui(new Ui::SimCameraWidget),
    m_camera(camera)
{
    m_ui->setupUi(this);

    //TODO set ID
    m_ui->fpsvalue->setValue(m_camera->getParams().maxFPS);
}

SimCameraWidget::~SimCameraWidget()
{
    delete m_ui;
}

void SimCameraWidget::setID(int id)
{
    m_ui->cameraidvalue->setText(QString::number(id));
}

void SimCameraWidget::on_fpsvalue_valueChanged(int arg1)
{
    m_camera->getParams().maxFPS = arg1;
}

void SimCameraWidget::on_xvalue_valueChanged(double arg1)
{
    m_camera->getParams().translation[0] = arg1;
}

void SimCameraWidget::on_yvalue_valueChanged(double arg1)
{
    m_camera->getParams().translation[1] = arg1;
}

void SimCameraWidget::on_zvalue_valueChanged(double arg1)
{
    m_camera->getParams().translation[2] = arg1;
}

void SimCameraWidget::on_rotxvalue_valueChanged(double arg1)
{
    m_camera->getParams().rotation[0] = arg1 * M_PI/180.0;
}

void SimCameraWidget::on_rotyvalue_valueChanged(double arg1)
{
    m_camera->getParams().rotation[1] = arg1 * M_PI/180.0;
}

void SimCameraWidget::on_rotzvalue_valueChanged(double arg1)
{
    m_camera->getParams().rotation[2] = arg1 * M_PI/180.0;
}

}
