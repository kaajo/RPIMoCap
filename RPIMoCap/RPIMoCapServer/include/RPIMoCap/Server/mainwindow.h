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
#include "camerasettingswidget.h"

#include <RPIMoCap/Core/line3d.h>

#include <QMainWindow>
#include <QVector>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

signals:
    void startMoCap(bool start);
    void startCalib(bool start);

public slots:
    void onLinesReceived(const std::vector<RPIMoCap::Line3D> &lines);
    void addCamera(const std::shared_ptr<CameraSettings> &camera);
    void removeCamera(const int id);

private slots:
    void on_MoCapButton_clicked(bool checked);
    void on_calibButton_clicked(bool checked);

private:
    Ui::MainWindow *ui;

    QMap<int, CameraSettingsWidget*> m_cameraWidgets;
};
