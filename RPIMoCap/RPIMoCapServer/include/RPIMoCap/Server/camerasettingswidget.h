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

#include <Eigen/Geometry>

namespace Ui {
class CameraSettingsWidget;
}

namespace RPIMoCap {

class CameraSettingsWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CameraSettingsWidget(QUuid id, QWidget *parent = nullptr);
    virtual ~CameraSettingsWidget();

    Visualization::ExtrinsicWidget *extrinsic();

private:
    Ui::CameraSettingsWidget *ui;
};

}
