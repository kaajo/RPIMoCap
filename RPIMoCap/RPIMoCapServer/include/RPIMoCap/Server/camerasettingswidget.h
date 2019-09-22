#pragma once

#include "camerasettings.h"

#include <QWidget>

#include <eigen3/Eigen/Geometry>

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

