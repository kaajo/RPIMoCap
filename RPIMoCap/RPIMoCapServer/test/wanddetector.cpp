#include <gtest/gtest.h>

#include <RPIMoCap/Server/wanddetector.h>
#include <RPIMoCap/Core/cameraparams.h>

#include <opencv2/core/affine.hpp>
#include <opencv2/calib3d.hpp>

RPIMoCap::CameraParams v1params = RPIMoCap::CameraParams::computeRPICameraV1Params();

std::vector<cv::Point3f> create3PWand(float sizecm, float middlePointOffsetcm)
{
    std::vector<cv::Point3f> points;
    points.emplace_back(-sizecm/2.0f, 0.0f, 0.0f);
    points.emplace_back(middlePointOffsetcm, 0.0f, 0.0f);
    points.emplace_back(sizecm/2.0f, 0.0f, 0.0f);

    return points;
}

TEST(wanddetector, 3Pfalse)
{
    EXPECT_FALSE(WandDetector::detect3pWand({}).has_value());
}

TEST(wanddetector, 3Psimple)
{
    auto wandPoints = create3PWand(50.0f, 10.0f);

    cv::Vec3f rVec(0.0f, 0.0f, 0.0f);
    cv::Vec3f tVec(0.0f, 0.0, 200.0f);

    for (size_t i = 0; i < wandPoints.size(); ++i)
    {
        wandPoints[i] = cv::Affine3f(rVec, tVec) * wandPoints[i];
    }

    std::vector<cv::Point2f> pixels;
    cv::projectPoints(wandPoints, cv::Vec3f::zeros(), cv::Vec3f::zeros(),
                      v1params.cameraMatrix, v1params.distortionCoeffs, pixels);

    auto detection = WandDetector::detect3pWand(pixels);

    EXPECT_TRUE(detection.has_value());

    std::vector<cv::Point2f> detectedPixels = detection.value();

    for (int i = 0; i < 3; ++i)
    {
        EXPECT_EQ(detectedPixels[i], pixels[i]);
    }
}
