#include "Server/wanddetector.h"

#include <opencv2/core.hpp>
#include <QSet>

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
