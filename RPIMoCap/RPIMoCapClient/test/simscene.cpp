#include <gtest/gtest.h>

#include <RPIMoCap/SimClient/simscene.h>
#include <RPIMoCap/Core/cameraparams.h>

#include <opencv2/core/affine.hpp>
#include <opencv2/calib3d.hpp>

using RPIMoCap::SimClient::SimScene;
using RPIMoCap::SimClient::SimMarker;

TEST(simscene, identity)
{
    SimMarker marker;
    marker.translation = {0.0, 0.0, 200.0};

    SimScene scene;
    scene.setMarkers({marker});

    RPIMoCap::CameraParams v1params = RPIMoCap::CameraParams::computeRPICameraV1Params();
    v1params.translation = {0.0, 0.0, 0.0};

    cv::Mat image = scene.projectScene(v1params);

    EXPECT_EQ(image.at<uint8_t>(240, 320), 255);
}

TEST(simscene, cameraTranslationX)
{    
    SimMarker marker;
    marker.translation = {20.0, 0.0, 200.0};

    SimScene scene;
    scene.setMarkers({marker});

    RPIMoCap::CameraParams v1params = RPIMoCap::CameraParams::computeRPICameraV1Params();
    v1params.translation = {20.0, 0.0, 0.0};
    v1params.rotation = {0.0, 0.0, 0.0};

    cv::Mat image = scene.projectScene(v1params);

    EXPECT_EQ(image.at<uint8_t>(240, 320), 255);
}

TEST(simscene, cameraRotationX)
{
    SimMarker marker;
    marker.translation = {200.0, 0.0, 0.0};

    SimScene scene;
    scene.setMarkers({marker});

    RPIMoCap::CameraParams v1params = RPIMoCap::CameraParams::computeRPICameraV1Params();
    v1params.translation = {0.0, 0.0, 0.0};
    v1params.rotation = {0.0, M_PI/2.0, 0.0};

    cv::Mat image = scene.projectScene(v1params);

    EXPECT_EQ(image.at<uint8_t>(240, 320), 255);
}

TEST(simscene, cameraTRX)
{
    SimMarker marker;
    marker.translation = {200.0, 0.0, 20.0};

    SimScene scene;
    scene.setMarkers({marker});

    RPIMoCap::CameraParams v1params = RPIMoCap::CameraParams::computeRPICameraV1Params();
    v1params.translation = {0.0, 0.0, 20.0};
    v1params.rotation = {0.0, M_PI/2.0, 0.0};

    cv::Mat image = scene.projectScene(v1params);

    EXPECT_EQ(image.at<uint8_t>(240, 320), 255);
}
