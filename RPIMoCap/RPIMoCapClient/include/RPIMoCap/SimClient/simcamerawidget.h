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

#include <RPIMoCap/SimClient/simcamera.h>

#include <QWidget>

#include <memory>

namespace Ui {
class SimCameraWidget;
}

namespace RPIMoCap::SimClient {

class SimCameraWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SimCameraWidget(std::shared_ptr<SimCamera> camera, QWidget *parent = nullptr);
    ~SimCameraWidget();

private slots:
    void on_fpsvalue_valueChanged(int arg1);

private:
    Ui::SimCameraWidget *m_ui;
    std::shared_ptr<SimCamera> m_camera;
};

}
