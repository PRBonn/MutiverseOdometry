#pragma once
#include "common_vlo.hpp"

namespace vlo {
cv::Point2d projectPointToImagePlane(const cv::Point3d &p, const cv::Mat &K);
cv::Point2d projectPointToImagePlane(const Vec3 &p, const cv::Mat &K);
bool isInImage(const cv::Mat &image, const cv::Point2d &pixel) ;
}  // namespace vlo
