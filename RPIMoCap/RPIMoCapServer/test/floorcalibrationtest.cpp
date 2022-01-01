#include <gtest/gtest.h>

#include <Server/Calibration/floorcalibration.h>

#include <iostream>


#include <opencv2/calib3d/calib3d.hpp>

using namespace RPIMoCap;

std::vector<cv::Point2f> projectPoints(const Camera::Intrinsics& intrinsic,
                                       const Eigen::Affine3f& camTransform,
                                       const std::vector<Eigen::Vector3f>& points) {

    std::vector<cv::Point2f> px;

    std::vector<cv::Vec3f> pointsInCamera;
    std::transform(points.begin(), points.end(), std::back_inserter(pointsInCamera),
                   [camTransform](const Eigen::Vector3f &point)
                   {const Eigen::Vector3f transformed = camTransform.inverse() * point;
                    return cv::Vec3f(transformed.x(), transformed.y(), transformed.z());});

    cv::projectPoints(pointsInCamera, cv::Vec3f::zeros(), cv::Vec3f::zeros(),
                      intrinsic.cameraMatrix, intrinsic.distortionCoeffs, px);

    return px;
}

TEST(floorcalibration, identity)
{
    std::vector<FloorCalibration::CameraData> inputData;

    auto intrinsic = Camera::Intrinsics::computeRPICameraV1Params();

    float sizecm = 10.0f;

    std::vector<Eigen::Vector3f> points;
    points.push_back({0.0f, 0.0f, 0.0f});
    points.push_back({sizecm, 0.0f, 0.0f});
    points.push_back({-sizecm, 0.0f, 0.0f});
    points.push_back({0.0f, 0.0f, -sizecm});
    points.push_back({0.0f, 0.0f, sizecm});

    // First camera
    {
        QUuid uuid = QUuid::createUuid();
        Eigen::Affine3f extrinsics = Eigen::Affine3f::Identity();
        extrinsics.translation() = Eigen::Vector3f(-10.0f, 20.0f, -100.0f);

        std::vector<cv::Point2f> px = projectPoints(intrinsic, extrinsics, points);
        std::vector<Line3D> rays;
        // create rays
        for (size_t i = 0; i < points.size(); ++i) {
            rays.push_back(Line3D(extrinsics.translation(), (points[i] - extrinsics.translation()).normalized()));
        }
        inputData.push_back({uuid, intrinsic, /*extrinsics,*/ px, rays});
    }

    // Second camera
    {
        QUuid uuid = QUuid::createUuid();
        Eigen::Affine3f extrinsics;
        extrinsics.translation() = Eigen::Vector3f(10.0f, 20.0f, -100.0f);

        std::vector<cv::Point2f> px = projectPoints(intrinsic, extrinsics, points);
        std::vector<Line3D> rays;
        // create rays
        for (size_t i = 0; i < points.size(); ++i) {
            rays.push_back(Line3D(extrinsics.translation(), (points[i] - extrinsics.translation()).normalized()));
        }
        inputData.push_back({uuid, intrinsic, /*extrinsics,*/ px, rays});
    }

    auto transform = FloorCalibration::computeOrigin(inputData, sizecm);
    EXPECT_TRUE(transform.has_value());
    EXPECT_TRUE(transform->translation().norm()  < 0.1);
    // TODO check rotation
}
