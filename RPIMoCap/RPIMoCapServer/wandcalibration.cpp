#include "wandcalibration.h"

#include <opencv2/calib3d.hpp>

WandCalibration::WandCalibration(QObject *parent) : QObject(parent)
{

}

float WandCalibration::addPoints(size_t firstCamID, std::vector<std::vector<cv::Point2f>> firstPoints,
                                 size_t secondCamID, std::vector<std::vector<cv::Point2f>> secondPoints)
{
    assert(firstPoints.size() == secondPoints.size());

    //filter all incorrect frames
    for (size_t i = firstPoints.size()-1; i >= 0; --i)
    {
        if (firstPoints[i].size() != m_wandPoints.size() ||
            firstPoints[i].size() != secondPoints[i].size())
        {
            firstPoints.erase(firstPoints.begin() + i);
            secondPoints.erase(secondPoints.begin() + i);
        }
    }

    cv::Mat mask;

    cv::Mat eMatrix = cv::findEssentialMat(firstPoints,secondPoints, cv::Mat() /*TODO*/, cv::RANSAC, 0.999, 1.0, mask);

    cv::Mat rotation;
    cv::Mat translation;

    cv::recoverPose(eMatrix, firstPoints, secondPoints, cv::Mat() /*TODO*/, rotation, translation, mask);


}
