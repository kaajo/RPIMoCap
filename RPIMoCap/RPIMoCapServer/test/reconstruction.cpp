#include <gtest/gtest.h>

#include <Server/camerasettings.h>

#include <iostream>

//TEST(essentialMatrix, basic)
//{
//    QUuid uid = QUuid::createUuid();
//    RPIMoCap::MQTTSettings settings;

//    RPIMoCap::Camera::Intrinsics camParams = RPIMoCap::Camera::Intrinsics::computeRPICameraV1Params();

//    RPIMoCap::CameraSettings leftCam(uid, settings, camParams);
//    leftCam.setTranslation({0.0, 100.0, -100.0});

//    RPIMoCap::CameraSettings rightCam(uid, settings, camParams);
//    rightCam.setTranslation({-60.0, 100.0, -100.0});

//    cv::Point2f pixelLeft(580, 224);
//    cv::Point2f pixelRight(505, 224);

//    cv::Point2f resRight = leftCam.findCorrespondentPixel(pixelLeft, rightCam);
//    EXPECT_FLOAT_EQ(resRight.x, pixelRight.x);
//    EXPECT_FLOAT_EQ(resRight.y, pixelRight.y);

//    cv::Point2f resLeft = rightCam.findCorrespondentPixel(pixelRight, leftCam);
//    EXPECT_FLOAT_EQ(resLeft.x, pixelLeft.x);
//    EXPECT_FLOAT_EQ(resLeft.y, pixelLeft.y);
//}

TEST(essentialMatrix, coperinopasterino)
{
    QUuid uid = QUuid::createUuid();
    RPIMoCap::MQTTSettings settings;
    RPIMoCap::Camera::Intrinsics camParams = RPIMoCap::Camera::Intrinsics::computeRPICameraV1Params();

    RPIMoCap::CameraSettings leftCam(uid, settings, camParams);
    RPIMoCap::CameraSettings rightCam(uid, settings, camParams);
    rightCam.setTranslation({1.0, 0.0, 0.0});

    cv::Point2f pixelLeft(512, 432); // (0.6f, 0.8f)
    cv::Point2f pixelRight(448, 432); // (0.4f, 0.8f)

    auto pixelLeftNorm = leftCam.normalizeCoords(pixelLeft);
    auto pixelRightNorm = rightCam.normalizeCoords(pixelRight);

    auto essentialMatrix = leftCam.essentialMatrix(rightCam);
    auto epipolarLine = leftCam.epipolarLine(pixelLeftNorm, essentialMatrix);

    double distance = epipolarLine.distance(Eigen::Vector2d(pixelRightNorm.x, pixelRightNorm.y));
    EXPECT_FLOAT_EQ(distance, 0.0f);

//    Eigen::Matrix<float, 3, 3> essentialMatrix = computeEssentialMatrix(rotationMatrix, translationVector);
//    Eigen::Matrix<float, 3, 1> leftImagePoint(0.6f, 0.8f, 1.f);
//    Eigen::Matrix<float, 3, 1> rightImagePoint(0.4f, 0.8f, 1.f);

//    Eigen::Matrix<float, 3, 1> rightEpipolarLine = computeRightEpipolarLine(essentialMatrix, leftImagePoint);

//    EXPECT_NEAR(0.f, rightEpipolarLine(0, 0), 1.0e-6f);
//    EXPECT_NEAR(1.f, rightEpipolarLine(1, 0), 1.0e-6f);
//    EXPECT_NEAR(-0.8f, rightEpipolarLine(2, 0), 1.0e-6f);

//    EXPECT_NEAR(0.f, shortestDistPointToLine(rightImagePoint, rightEpipolarLine), 1.0e-5f);
}
