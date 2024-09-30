#pragma once

#include <cmath>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace reuv::basic_components {

/**
 * @brief Get the intercept of a line given a slope and a point.
 * @param slope The slope of the line.
 * @param point The point on the line.
 * @return The intercept of the line.
 */
template <typename T>
inline T get_intercept(T slope, const cv::Point_<T>& point)
{
    return point.y - slope * point.x;
}

/**
 * @brief Extract a region of interest from an image given a set of points.
 * @param input_frame The input image.
 * @param points The points that define the ROI.
 * @return The extracted ROI.
 */
inline cv::Mat extract_ROI(const cv::Mat& image, const std::vector<cv::Point>& points)
{
    cv::Mat mask = cv::Mat::zeros(image.size(), image.type());
    cv::fillConvexPoly(mask, points, cv::Scalar(255, 255, 255));
    cv::Mat result;
    image.copyTo(result, mask);
    return result;
}

/**
 * @brief Get the distance between two points.
 * @param pointO The origin point.
 * @param pointA The target point.
 * @return The distance between the two points.
 */
template <typename T>
inline T get_distance(const cv::Point_<T>& pointO, const cv::Point_<T>& pointA)
{
    return sqrt(pow(pointA.x - pointO.x, 2) + pow(pointA.y - pointO.y, 2));
}

/**
 * @brief Get the slope of a line given two points.
 * @param pointA The first point.
 * @param pointB The second point.
 * @return The slope of the line.
 */
template <typename T>
inline T get_slope(const cv::Point_<T>& pointA, const cv::Point_<T>& pointB)
{
    return (pointB.y - pointA.y) / (pointB.x - pointA.x);
}
}
