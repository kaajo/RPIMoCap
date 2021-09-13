#include <gtest/gtest.h>

#include <Server/wandcalibration.h>

#include <iostream>

using RPIMoCap::ObservationPair;
using RPIMoCap::WandCalibration;

QVector<QUuid> generateUuids(size_t count)
{
    QVector<QUuid> ids;

    for (size_t i = 0; i < count; ++i)
    {
        ids.push_back(QUuid::createUuid());
    }

    return ids;
}

TEST(wandcalibration, identity)
{
    QMap<std::pair<QUuid, QUuid>, ObservationPair> detections;

    auto ids = generateUuids(2);

    ObservationPair d;
    detections.insert({ids[0], ids[1]}, d);

    auto transforms = WandCalibration::relativeToGlobalTransforms(detections);

    EXPECT_EQ(transforms.size(), 2);
    EXPECT_TRUE(transforms.find(ids[0]).value().isApprox(Eigen::Affine3f::Identity()));
    EXPECT_TRUE(transforms.find(ids[1]).value().isApprox(Eigen::Affine3f::Identity()));
}

TEST(wandcalibration, simple)
{
    QMap<std::pair<QUuid, QUuid>, ObservationPair> detections;

    auto ids = generateUuids(2);

    ObservationPair d;
    d.second.tVec = Eigen::Vector3f(1.0, 2.0, 3.0);

    detections.insert({ids[0], ids[1]}, d);

    auto transforms = WandCalibration::relativeToGlobalTransforms(detections);

    EXPECT_EQ(transforms.size(), 2);
    EXPECT_TRUE(transforms.find(ids[0]).value().isApprox(Eigen::Affine3f::Identity()));
    EXPECT_TRUE(transforms.find(ids[1]).value().isApprox(d.relativeTransform()));
}

TEST(wandcalibration, threeCamerasTranslation)
{
    QMap<std::pair<QUuid, QUuid>, ObservationPair> detections;

    auto ids = generateUuids(3);

    {
        ObservationPair d;
        d.second.tVec = Eigen::Vector3f(1.0, 0.0, 0.0);
        detections.insert({ids[0], ids[1]}, d);
    }

    {
        ObservationPair d;
        d.second.tVec = Eigen::Vector3f(1.0, 0.0, 0.0);
        detections.insert({ids[1], ids[2]}, d);
    }

    auto transforms = WandCalibration::relativeToGlobalTransforms(detections);

    Eigen::Affine3f expectedTransform = Eigen::Affine3f::Identity();
    expectedTransform.translate(Eigen::Vector3f(2.0, 0.0, 0.0));

    EXPECT_EQ(transforms.size(), 3);

    Eigen::Affine3f transform = transforms.find(ids[2]).value() * transforms.find(ids[0]).value().inverse();

    EXPECT_TRUE(transform.isApprox(expectedTransform));
}

TEST(wandcalibration, threeCamerasRotation)
{
    QMap<std::pair<QUuid, QUuid>, ObservationPair> detections;

    auto ids = generateUuids(3);

    for (size_t i = 0; i < 2; ++i)
    {
        ObservationPair d;
        d.second.rVec = Eigen::Vector3f(0.0, M_PI_2, 0.0);
        d.second.tVec = Eigen::Vector3f(1.0, 0.0, 0.0);
        detections.insert({ids[i], ids[i+1]}, d);
    }

    auto transforms = WandCalibration::relativeToGlobalTransforms(detections);

    EXPECT_EQ(transforms.size(), 3);

    Eigen::Affine3f expectedTransform = Eigen::Affine3f::Identity();
    expectedTransform.translate(Eigen::Vector3f(1.0, 0.0, -1.0));
    expectedTransform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY()));

    Eigen::Affine3f transform = transforms.find(ids[2]).value() * transforms.find(ids[0]).value().inverse();

    EXPECT_NEAR(transform.translation().norm(), std::sqrt(2), 0.0001);
    EXPECT_TRUE(transform.isApprox(expectedTransform));
}

TEST(wandcalibration, square)
{
    QMap<std::pair<QUuid, QUuid>, ObservationPair> detections;

    auto ids = generateUuids(5);

    for (size_t i = 0; i < 4; ++i)
    {
        ObservationPair d;
        d.second.rVec = Eigen::Vector3f(0.0, M_PI_2, 0.0);
        d.second.tVec = Eigen::Vector3f(1.0, 0.0, 0.0);
        detections.insert({ids[i], ids[i+1]}, d);
    }

    auto transforms = WandCalibration::relativeToGlobalTransforms(detections);

    EXPECT_EQ(transforms.size(), 5);
    EXPECT_TRUE(transforms.find(ids[0]).value().isApprox(transforms.find(ids[4]).value()));
}
