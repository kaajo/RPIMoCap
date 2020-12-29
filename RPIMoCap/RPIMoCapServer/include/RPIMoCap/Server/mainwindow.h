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

#include <RPIMoCap/Core/frame.h>

#include <QMainWindow>
#include <QVector>

namespace Ui {
class MainWindow;
}

namespace RPIMoCap {

class CalibrationWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    CalibrationWidget* calibrationWidget() {return m_calibWidget;};

signals:
    void startMoCap(bool start);
    void searchForCameras();

public slots:
    void addCamera(const std::shared_ptr<CameraSettings> &camera);
    void updateCamera();
    void removeCamera(const QUuid id);
    void drawFrame(const RPIMoCap::Frame &frame);

private slots:
    void on_MoCapButton_clicked(bool checked);

private:
    Ui::MainWindow *ui;

    CalibrationWidget *m_calibWidget = nullptr;

    QMap<QUuid, CameraSettingsWidget*> m_cameraWidgets;
};

}
