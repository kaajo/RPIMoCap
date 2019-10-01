#include "RPIMoCap/Server/camerasettingswidget.h"
#include "ui_camerasettingswidget.h"

CameraSettingsWidget::CameraSettingsWidget(const std::shared_ptr<CameraSettings> camera, QWidget *parent)
    : QWidget(parent)
    , m_camera(camera)
    , ui(new Ui::CameraSettingsWidget)
{
    ui->setupUi(this);
    ui->cameraid->setText(QString::number(camera->id()));
    connect(m_camera.get(), &CameraSettings::changed, this, &CameraSettingsWidget::setValues);
    setValues();
}

CameraSettingsWidget::~CameraSettingsWidget()
{
    delete ui;
}

void CameraSettingsWidget::on_pushButton_clicked()
{
    auto t = Eigen::Affine3f::Identity();

    const float rollRad = ui->rotationX->value();
    const float pitchRad = ui->rotationY->value();
    const float yawRad = ui->rotationZ->value();

    Eigen::Matrix3f rot;
    rot = Eigen::AngleAxisf(yawRad, Eigen::Vector3f::UnitZ())
        * Eigen::AngleAxisf(pitchRad, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(rollRad, Eigen::Vector3f::UnitX());

    t.rotate(rot);

    t.translation().x() = ui->translationX->value();
    t.translation().y() = ui->translationY->value();
    t.translation().z() = ui->translationZ->value();

    m_camera->setTransform(t);
}

void CameraSettingsWidget::setValues()
{
    const auto transVec = m_camera->transform().translation();
    ui->translationX->setValue(transVec.x());
    ui->translationY->setValue(transVec.y());
    ui->translationZ->setValue(transVec.z());

    const auto rotVec = m_camera->transform().rotation().eulerAngles(0,1,2);
    ui->rotationX->setValue(rotVec.x() * 180.0/M_PI);
    ui->rotationY->setValue(rotVec.y() * 180.0/M_PI);
    ui->rotationZ->setValue(rotVec.z() * 180.0/M_PI);
}
