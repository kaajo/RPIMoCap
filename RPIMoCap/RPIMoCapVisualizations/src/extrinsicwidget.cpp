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

#include "RPIMoCap/extrinsicwidget.h"
#include "ui_extrinsicwidget.h"

namespace RPIMoCap::Visualization {

constexpr double degToRad = M_PI/180.0;
constexpr double radToDeg = 180.0/M_PI;

ExtrinsicWidget::ExtrinsicWidget(QWidget *parent) :
    QWidget(parent),
    m_ui(new Ui::ExtrinsicWidget)
{
    m_ui->setupUi(this);
    connect(m_ui->rx,qOverload<double>(&QDoubleSpinBox::valueChanged), this, &ExtrinsicWidget::onRotationChange);
    connect(m_ui->ry,qOverload<double>(&QDoubleSpinBox::valueChanged), this, &ExtrinsicWidget::onRotationChange);
    connect(m_ui->rz,qOverload<double>(&QDoubleSpinBox::valueChanged), this, &ExtrinsicWidget::onRotationChange);
    connect(m_ui->tx,qOverload<double>(&QDoubleSpinBox::valueChanged), this, &ExtrinsicWidget::onTranslationChange);
    connect(m_ui->ty,qOverload<double>(&QDoubleSpinBox::valueChanged), this, &ExtrinsicWidget::onTranslationChange);
    connect(m_ui->tz,qOverload<double>(&QDoubleSpinBox::valueChanged), this, &ExtrinsicWidget::onTranslationChange);
}

ExtrinsicWidget::~ExtrinsicWidget()
{
    delete m_ui;
}

cv::Vec3f ExtrinsicWidget::getRotation()
{
    float x = m_ui->rx->value() * degToRad;
    float y = m_ui->ry->value() * degToRad;
    float z = m_ui->rz->value() * degToRad;
    return {x, y, z};
}

cv::Vec3f ExtrinsicWidget::getTranslation()
{
    float x = m_ui->tx->value();
    float y = m_ui->ty->value();
    float z = m_ui->tz->value();
    return {x, y, z};
}

Eigen::Affine3f ExtrinsicWidget::getTransform()
{
    auto rVec = getRotation();
    auto tVec = getTranslation();

    Eigen::Matrix3f rot;
    rot = Eigen::AngleAxisf(rVec[0], Eigen::Vector3f::UnitX())
          * Eigen::AngleAxisf(rVec[1], Eigen::Vector3f::UnitY())
          * Eigen::AngleAxisf(rVec[2], Eigen::Vector3f::UnitZ());

    Eigen::Vector3f translation(tVec[0], tVec[1], tVec[2]);

    Eigen::Affine3f t = Eigen::Affine3f::Identity();
    t.fromPositionOrientationScale(translation, rot, Eigen::Vector3f(1.0, 1.0, 1.0));
    return t;
}

void ExtrinsicWidget::setRotation(cv::Vec3f rVec)
{
    m_ui->rx->setValue(rVec[0] * radToDeg);
    m_ui->ry->setValue(rVec[1] * radToDeg);
    m_ui->rz->setValue(rVec[2] * radToDeg);
}

void ExtrinsicWidget::setTranslation(cv::Vec3f tVec)
{
    m_ui->tx->setValue(tVec[0]);
    m_ui->ty->setValue(tVec[1]);
    m_ui->tz->setValue(tVec[2]);
}

void ExtrinsicWidget::onRotationChange()
{
    emit rotationChanged(getRotation());
    emit transformChanged(getTransform());
}

void ExtrinsicWidget::onTranslationChange()
{
    emit translationChanged(getTranslation());
    emit transformChanged(getTransform());
}

}
