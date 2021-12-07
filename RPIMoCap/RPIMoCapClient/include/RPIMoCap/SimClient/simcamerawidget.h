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

#pragma once

#include <RPIMoCap/extrinsicwidget.h>

#include <QWidget>
#include <QUuid>

#include <opencv2/core/mat.hpp>
#include <Eigen/Geometry>

#include <memory>

namespace Ui {
class SimCameraWidget;
}

namespace RPIMoCap::SimClient {

class SimCameraWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SimCameraWidget(uint16_t fps, QUuid clientId, QWidget *parent = nullptr);
    ~SimCameraWidget() override;

    Visualization::ExtrinsicWidget *extrinsic();

signals:
    void fpsChanged(QUuid clientid, uint16_t fps);
    void rotationChanged(QUuid clientId, Eigen::Vector3d rVec);
    void translationChanged(QUuid clientId, Eigen::Vector3d tVec);
    void transformChanged(QUuid clientId, Eigen::Affine3f transform);

private slots:
    void onfpsChanged(int fps);
    void onRotationChanged(Eigen::Vector3d rVec);
    void onTranslationChanged(Eigen::Vector3d tVec);
    void onTransformChanged(Eigen::Affine3f transform);

private:
    Ui::SimCameraWidget *m_ui = nullptr;

    QUuid m_clientId;
};

}
