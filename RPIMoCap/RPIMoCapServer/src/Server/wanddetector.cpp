#include "Server/wanddetector.h"

#include <opencv2/core.hpp>
#include <QSet>

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

std::optional<std::vector<cv::Point2f> > WandDetector::detect4pWand(const std::vector<cv::Point2f> &pts)
{
    if (pts.size() != 4)
    {
        return std::nullopt;
    }

    cv::Mat distMap(pts.size(), pts.size(), CV_32FC1,
                    cv::Scalar(std::numeric_limits<float>::infinity()));

    for (size_t i = 0; i < pts.size(); ++i)
    {
        for (size_t j = i+1; j < pts.size(); ++j)
        {
            distMap.at<float>(i,j) = distMap.at<float>(j,i) = cv::norm(pts[i] - pts[j]);
        }
    }

    double min = std::numeric_limits<double>::lowest();
    double max = std::numeric_limits<double>::max();
    cv::Point minLoc(-1, -1);
    cv::Point maxLoc(-1, -1);
    cv::Mat diagMask = cv::Scalar::all(1) - cv::Mat::eye(pts.size(), pts.size(), CV_8UC1);
    cv::minMaxLoc(distMap, &min, &max, &minLoc, &maxLoc, diagMask);

    size_t borderL = maxLoc.x;
    size_t borderR = maxLoc.y;
    size_t middle = minLoc.x;
    size_t cross = minLoc.y;

    if (distMap.at<float>(cross,borderL) <  distMap.at<float>(middle,borderL))
    {
        std::swap(cross, middle);
    }

    if (distMap.at<float>(middle,borderL) <  distMap.at<float>(middle,borderR))
    {
        std::swap(borderL, borderR);
    }

    return std::vector<cv::Point2f>{pts[borderL], pts[middle], pts[borderR], pts[cross]};
}

std::optional<std::vector<cv::Point2f> > WandDetector::detect3pWand(const std::vector<cv::Point2f> &pts)
{
    if (pts.size() != 3)
    {
        return std::nullopt;
    }

    std::vector<float> distances;
    distances.push_back(cv::norm(pts[1] - pts[0]) + cv::norm(pts[2] - pts[0]));
    distances.push_back(cv::norm(pts[0] - pts[1]) + cv::norm(pts[2] - pts[1]));
    distances.push_back(cv::norm(pts[0] - pts[2]) + cv::norm(pts[1] - pts[2]));

    //TODO take also wand points for correct order
    const size_t middleID = (std::min_element(distances.begin(), distances.end()) - distances.begin());
    const size_t leftID = (std::max_element(distances.begin(), distances.end()) - distances.begin());

    QSet<size_t> indexes{0, 1, 2};
    indexes.remove(middleID);
    indexes.remove(leftID);
    const size_t rightID = indexes.values().first();

    return std::vector<cv::Point2f>{pts[leftID], pts[middleID], pts[rightID]};
}

std::optional<WandDetector::CrossDetection> WandDetector::detectCross(const std::vector<cv::Point3f> &pts, const float size)
{
    if (pts.size() != 5)
    {
        return std::nullopt;
    }

    std::vector<float> RMSE;

    for (size_t i = 0; i < 5; ++i) {

        float RMSEI = 0.0f;

        for (size_t j = 0; j < 5; ++j) {
            RMSEI += std::pow(cv::norm(pts[i] - pts[j]) - size, 2);
        }

        RMSE.push_back(sqrtf(1.0f/5.0f * RMSEI));
    }

    auto centerIndex = std::min_element(RMSE.begin(), RMSE.end()) - RMSE.begin();

    WandDetector::CrossDetection detection;
    detection.centerPoint = pts[centerIndex];

    for (long i = 0; i < 5; ++i) {
        if (i != centerIndex) {
            detection.edgePoints.push_back(pts[i]);
        }
    }

    // Compute normals
    Eigen::Map<const Eigen::Matrix3Xf> P(&pts[0].x, 3, pts.size());

    Eigen::Vector3f centroid = P.rowwise().mean();
    Eigen::MatrixXf centered = P.colwise() - centroid;
    Eigen::Matrix3f cov = centered * centered.transpose();

    //eigvecs sorted in increasing order of eigvals
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig;
    //eig.compute(cov);
    eig.computeDirect(cov);
    Eigen::Vector3f normal = eig.eigenvectors().col(0); //is already normalized
    if (normal(2) < 0.0f) normal = -normal; //flip towards camera

    detection.normalVector = {normal.x(), normal.y(), normal.z()};

    return detection;
}
