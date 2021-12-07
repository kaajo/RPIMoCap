#include "camerasettings.h"

namespace RPIMoCap {

CameraSettings::CameraSettings(QUuid id, const MQTTSettings &settings, Camera::Intrinsics camParams)
    : m_id(id)
    , m_pointSub("serverPointsSub-" + RPIMoCap::MQTTTopics::uuidString(id),
                 RPIMoCap::MQTTTopics::pixels(id),settings)
    , m_params(std::move(camParams))
{
    connect(&m_pointSub,&RPIMoCap::MQTTSubscriber::messageReceived, this, &CameraSettings::onPointsDataReceived);
}

Eigen::Affine3f CameraSettings::transform() const
{
    Eigen::Affine3f t = Eigen::Affine3f::Identity();

    Eigen::Matrix3f rot;
    rot = Eigen::AngleAxisf(m_rotation[0], Eigen::Vector3f::UnitX())
          * Eigen::AngleAxisf(m_rotation[1], Eigen::Vector3f::UnitY())
          * Eigen::AngleAxisf(m_rotation[2], Eigen::Vector3f::UnitZ());
    t.rotate(rot);

    t.translation().x() = m_translation[0];
    t.translation().y() = m_translation[1];
    t.translation().z() = m_translation[2];

    return t;
}

Line3D CameraSettings::computePixelRay(cv::Point2f pixel)
{
    cv::Mat normalized = m_params.cameraMatrixInv * cv::Mat(cv::Vec3f(pixel.x, pixel.y, 1));

    Eigen::Vector3f origin(m_translation[0], m_translation[1], m_translation[2]);
    auto dir = cv::Affine3f(cv::Vec3f(m_rotation.x(), m_rotation.y(), m_rotation.z())) * cv::Vec3f(normalized.at<float>(0), normalized.at<float>(1), normalized.at<float>(2));

    return Line3D(origin, Eigen::Vector3f(dir[0], dir[1], dir[2]).normalized()); //TODO more effective!!!
}

Eigen::Matrix<double, 3, 3> CameraSettings::essentialMatrix(const CameraSettings &targetCamera)
{
    Eigen::AngleAxisd RXS(this->m_rotation.x(), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd RYS(this->m_rotation.y(), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd RZS(this->m_rotation.z(), Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond qS = RZS * RYS * RXS;

    Eigen::AngleAxisd RXT(targetCamera.m_rotation.x(), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd RYT(targetCamera.m_rotation.y(), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd RZT(targetCamera.m_rotation.z(), Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond qT = RZT * RYT * RXT;

    auto R1 = qS.matrix();
    auto t1 = this->m_translation;
    auto R2 = qT.matrix();
    auto t2 = targetCamera.m_translation;

    Eigen::Matrix<double, 3, 3> R = R2 * R1.transpose();
    Eigen::Vector3d t = t2 - R * t1;
    Eigen::Matrix<double, 3, 3> Tx;
    Tx << 0, -t(2), t(1),  t(2), 0, -t(0),  -t(1), t(0), 0;

    return Tx * R;
}

cv::Point2f CameraSettings::normalizeCoords(cv::Point2f pixel)
{
    return cv::Point2f((pixel.x-320.0)/320.0f, (pixel.y-240.0)/240.0f);
}

Eigen::ParametrizedLine<double, 2> CameraSettings::epipolarLine(cv::Point2f pixelNormalized, const Eigen::Matrix<double,3, 3> &essentialMatrix)
{
    Eigen::Vector3d line = Eigen::Vector3d(pixelNormalized.x, pixelNormalized.y, 1).transpose() * essentialMatrix;

    const double x_min = std::min(-1.0, std::max(0.0, -line[2]/line[0])); // x coord of intersection with y=0
    const double x_max = std::min(1.0, std::max(0.0, (-line[2]-line[1]*1.0)/line[0])); // x coord of intersection with y=rows

    Eigen::Vector2d p1(x_min, (-line[2]-line[0]*x_min)/line[1]); // y= (-c-ax)/b
    Eigen::Vector2d p2(x_max, (-line[2]-line[0]*x_max)/line[1]);

    return Eigen::ParametrizedLine<double, 2>(p1,p2 - p1);
}

void CameraSettings::setTranslation(Eigen::Vector3d tVec)
{
    m_translation = tVec;
    emit translationChanged(tVec);
}

void CameraSettings::setRotation(Eigen::Vector3d rVec)
{
    m_rotation = rVec;
    emit rotationChanged(rVec);
}

void CameraSettings::onPointsDataReceived(const QByteArray &data)
{
    msgpack::object_handle result;
    msgpack::unpack(result, data.data(), data.length());

    const std::vector<cv::Point2f> points(result.get().as<std::vector<cv::Point2f>>());

    std::vector<std::pair<cv::Point2f, Line3D>> rays;
    for (auto &pnt : points)
    {
        rays.push_back({pnt,computePixelRay(pnt)});
    }

    emit raysReceived(m_id, rays);
}


Eigen::Matrix<double, 3, 3> CameraSettings::cameraMatFromCV(const cv::Mat camMatrix) {
    Eigen::Matrix<double, 3, 3> eigenMat;

    for (size_t i = 0; i < 9; ++i) {
        eigenMat(i) = static_cast<double>(camMatrix.at<float>(i));
    }

    return eigenMat;
}

}
