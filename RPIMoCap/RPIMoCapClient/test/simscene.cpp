#include <gtest/gtest.h>

#include <RPIMoCap/SimClient/simscene.h>
#include <RPIMoCap/Core/cameraparams.h>

#include <opencv2/core/affine.hpp>
#include <opencv2/calib3d.hpp>

using RPIMoCap::SimClient::SimScene;
using RPIMoCap::SimClient::SimMarker;
using RPIMoCap::Camera::Intrinsics;

TEST(simscene, identity)
{
    SimMarker marker;
    marker.translation = {0.0, 0.0, 200.0};

    SimScene scene;
    scene.setMarkers({marker});

    const Intrinsics v1params = Intrinsics::computeRPICameraV1Params();
    const cv::Vec3f rVec(0.0, 0.0, 0.0);
    const cv::Vec3f tVec(0.0, 0.0, 0.0);

    const cv::Mat image = scene.projectScene(v1params, rVec, tVec);

    EXPECT_EQ(image.at<uint8_t>(240, 320), 255);
}

TEST(simscene, cameraTranslationX)
{    
    SimMarker marker;
    marker.translation = {20.0, 0.0, 200.0};

    SimScene scene;
    scene.setMarkers({marker});

    const Intrinsics v1params = Intrinsics::computeRPICameraV1Params();
    const cv::Vec3f rVec(0.0, 0.0, 0.0);
    const cv::Vec3f tVec(20.0, 0.0, 0.0);

    const cv::Mat image = scene.projectScene(v1params, rVec, tVec);

    EXPECT_EQ(image.at<uint8_t>(240, 320), 255);
}

TEST(simscene, cameraRotationX)
{
    SimMarker marker;
    marker.translation = {200.0, 0.0, 0.0};

    SimScene scene;
    scene.setMarkers({marker});

    const Intrinsics v1params = Intrinsics::computeRPICameraV1Params();
    const cv::Vec3f rVec(0.0, M_PI/2.0, 0.0);
    const cv::Vec3f tVec(00.0, 0.0, 0.0);

    const cv::Mat image = scene.projectScene(v1params, rVec, tVec);

    EXPECT_EQ(image.at<uint8_t>(240, 320), 255);
}

TEST(simscene, cameraTRX)
{
    SimMarker marker;
    marker.translation = {200.0, 0.0, 20.0};

    SimScene scene;
    scene.setMarkers({marker});

    const Intrinsics v1params = Intrinsics::computeRPICameraV1Params();
    const cv::Vec3f rVec(0.0, M_PI/2.0, 0.0);
    const cv::Vec3f tVec(00.0, 0.0, 20.0);

    const cv::Mat image = scene.projectScene(v1params, rVec, tVec);

    EXPECT_EQ(image.at<uint8_t>(240, 320), 255);
}
