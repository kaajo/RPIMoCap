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

#include <QWidget>

#include <opencv2/core/mat.hpp>
#include <Eigen/Geometry>

#include <memory>

namespace Ui {
class ExtrinsicWidget;
}

namespace RPIMoCap::Visualization {

class ExtrinsicWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ExtrinsicWidget(QWidget *parent = nullptr);
    ~ExtrinsicWidget() override;

    Eigen::Vector3d getRotation();
    Eigen::Vector3d getTranslation();
    Eigen::Affine3f getTransform();

signals:
    void rotationChanged(Eigen::Vector3d rVec);
    void translationChanged(Eigen::Vector3d tVec);
    void transformChanged(Eigen::Affine3f transform);

public slots:
    void setRotation(Eigen::Vector3d rVec);
    void setTranslation(Eigen::Vector3d tVec);

private slots:
    void onRotationChange();
    void onTranslationChange();

private:
    Ui::ExtrinsicWidget *m_ui = nullptr;
};

}
