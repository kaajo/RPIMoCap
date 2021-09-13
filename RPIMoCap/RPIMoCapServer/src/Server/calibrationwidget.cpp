#include "Server/calibrationwidget.h"
#include "ui_calibrationwidget.h"

namespace RPIMoCap {

CalibrationWidget::CalibrationWidget(QWidget *parent) :
    QWidget(parent),
    m_ui(new Ui::CalibrationWidget)
{
    m_ui->setupUi(this);

    connect(m_ui->calibButton, &QPushButton::clicked, this, &CalibrationWidget::onCalibButton);
}

CalibrationWidget::~CalibrationWidget()
{
    delete m_ui;
}

void CalibrationWidget::onCalibFinished()
{
    m_ui->calibFrames->setEnabled(true);
    m_ui->calibType->setEnabled(true);
}

void CalibrationWidget::onCalibButton(bool checked)
{
    m_ui->calibButton->setText(checked ? "Stop" : "Start");
    m_ui->calibFrames->setEnabled(!checked);
    m_ui->calibType->setEnabled(!checked);

    WandCalibration::Settings settings;
    settings.framesPerCamera = m_ui->calibFrames->value();
    settings.calibType = WandCalibration::Type::Full;

    const QString typeText = m_ui->calibType->currentText();

    if (typeText == "Refine")
    {
        settings.calibType = WandCalibration::Type::Refine;
    }

    emit startCalibration(checked, settings);
}

} // namespace RPIMoCap
