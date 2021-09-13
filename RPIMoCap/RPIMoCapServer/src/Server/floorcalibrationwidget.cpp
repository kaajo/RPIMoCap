#include "floorcalibrationwidget.h"
#include "ui_floorcalibrationwidget.h"

namespace RPIMoCap {

FloorCalibrationWidget::FloorCalibrationWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FloorCalibrationWidget)
{
    ui->setupUi(this);
    QObject::connect(ui->takeSnapshot, &QPushButton::clicked, this, &FloorCalibrationWidget::onSnapshotButton);
}

FloorCalibrationWidget::~FloorCalibrationWidget()
{
    delete ui;
}

void FloorCalibrationWidget::onSnapshotButton()
{
    emit snapshotRequest(ui->offsetValue->value());
}

}
