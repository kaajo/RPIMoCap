#pragma once

#include <QWidget>

namespace Ui {
class FloorCalibrationWidget;
}

namespace RPIMoCap {

class FloorCalibrationWidget : public QWidget
{
    Q_OBJECT


public:
    explicit FloorCalibrationWidget(QWidget *parent = nullptr);
    ~FloorCalibrationWidget();


Q_SIGNALS:
    void snapshotRequest(const float offsetCM);

private Q_SLOTS:
    void onSnapshotButton();

private:
    Ui::FloorCalibrationWidget *ui;
};
}
