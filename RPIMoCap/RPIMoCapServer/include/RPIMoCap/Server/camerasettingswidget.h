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

#include "camerasettings.h"

#include <QWidget>

#include <Eigen/Geometry>

namespace Ui {
class CameraSettingsWidget;
}

class CameraSettingsWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CameraSettingsWidget(const std::shared_ptr<CameraSettings> camera, QWidget *parent = nullptr);
    virtual ~CameraSettingsWidget();

private slots:
    void on_pushButton_clicked();

    void setValues();
private:
    std::shared_ptr<CameraSettings> m_camera;
    Ui::CameraSettingsWidget *ui;
};

