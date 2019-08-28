#include "camerasettingswidget.h"
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

    {
        bool ok = false;
        float newVal = ui->translationXedit->text().toFloat(&ok);
        if (ok)
        {
            t.translation().x() = newVal;
        }
    }
    {
        bool ok = false;
        float newVal = ui->translationYedit->text().toFloat(&ok);
        if (ok)
        {
            t.translation().y() = newVal;
        }
    }
    {
        bool ok = false;
        float newVal = ui->translationZedit->text().toFloat(&ok);
        if (ok)
        {
            t.translation().z() = newVal;
        }
    }

    float rollRad = 0.0f;
    {
        bool ok = false;
        float newVal = ui->rotationXedit->text().toFloat(&ok);
        if (ok)
        {
            rollRad = newVal * M_PI/180.0;
        }
    }

    float pitchRad = 0.0f;
    {
        bool ok = false;
        float newVal = ui->rotationYedit->text().toFloat(&ok);
        if (ok)
        {
            pitchRad = newVal * M_PI/180.0;
        }
    }

    float yawRad = 0.0f;
    {
        bool ok = false;
        float newVal = ui->rotationZedit->text().toFloat(&ok);
        if (ok)
        {
            yawRad = newVal * M_PI/180.0;
        }
    }

    Eigen::Matrix3f rot;
    rot = Eigen::AngleAxisf(yawRad, Eigen::Vector3f::UnitZ())
        * Eigen::AngleAxisf(pitchRad, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(rollRad, Eigen::Vector3f::UnitX());

    t.rotate(rot);

    m_camera->setTransform(t);
}

void CameraSettingsWidget::setValues()
{
    const auto transVec = m_camera->transform().translation();
    ui->translationXedit->setText(QString::number(transVec.x()));
    ui->translationYedit->setText(QString::number(transVec.y()));
    ui->translationZedit->setText(QString::number(transVec.z()));

    const auto rotVec = m_camera->transform().rotation().eulerAngles(0,1,2);
    ui->rotationXedit->setText(QString::number(rotVec.x() * 180.0/M_PI));
    ui->rotationYedit->setText(QString::number(rotVec.y() * 180.0/M_PI));
    ui->rotationZedit->setText(QString::number(rotVec.z() * 180.0/M_PI));
}
