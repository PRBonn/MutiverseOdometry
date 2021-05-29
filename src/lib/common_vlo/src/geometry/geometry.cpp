#include "geometry/geometry.hpp"
namespace vlo{


cv::Point2d projectPointToImagePlane(const cv::Point3d &p, const cv::Mat &K) {
    return cv::Point2d(K.at<double>(0, 0) * p.x / p.z + K.at<double>(0, 2),
                       K.at<double>(1, 1) * p.y / p.z + K.at<double>(1, 2));
}

cv::Point2d projectPointToImagePlane(const Vec3 &p, const cv::Mat &K) {
    return cv::Point2d(K.at<double>(0, 0) * p[0] / p[2] + K.at<double>(0, 2),
                       K.at<double>(1, 1) * p[1] / p[2] + K.at<double>(1, 2));
}
bool isInImage(const cv::Mat &image, const cv::Point2d &pixel) {
    return pixel.x >= 0 && pixel.y >= 0 && pixel.x <= image.cols - 1 &&
           pixel.y <= image.rows - 1;
}
}
