#pragma once

#include "Server/Calibration/wandcalibration.h"

#include <QWidget>

namespace Ui {
class CalibrationWidget;
}

namespace RPIMoCap {

class CalibrationWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CalibrationWidget(QWidget *parent = nullptr);
    ~CalibrationWidget();

signals:
    void startCalibration(bool start, WandCalibration::Settings settings);

public slots:
    void onCalibFinished();

private slots:
    void onCalibButton(bool checked);

private:
    Ui::CalibrationWidget *m_ui = nullptr;
};

}
