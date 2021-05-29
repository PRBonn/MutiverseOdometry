#include "geometry/Camera.hpp"

namespace vlo {
// ---------------- transformation ----------------

// cv::Point2f cam2pixel(const cv::Mat &p, const cv::Mat &K) {
//  cv::Mat p0 = K * p;                     // project onto image
//  cv::Mat pp = p0 / p0.at<double>(2, 0);  // normalize
//  return cv::Point2f(pp.at<double>(0, 0), pp.at<double>(1, 0));
//}

cv::Point2d projectPointToImagePlane(const cv::Point3d &p, const cv::Mat &K) {
  return cv::Point2d(K.at<double>(0, 0) * p.x / p.z + K.at<double>(0, 2),
                     K.at<double>(1, 1) * p.y / p.z + K.at<double>(1, 2));
}

cv::Point2d projectPointToImagePlane(const Vec3 &p, const cv::Mat &K) {
  return cv::Point2d(K.at<double>(0, 0) * p[0] / p[2] + K.at<double>(0, 2),
                     K.at<double>(1, 1) * p[1] / p[2] + K.at<double>(1, 2));
}

Eigen::Vector3d getRGBvalueFromImageNormalized(const cv::Mat &img,
                                               const cv::Point2d &image_point) {
  Eigen::Vector3d bgr_normalized;
  auto bgr = img.at<cv::Vec3b>(image_point);
  bgr_normalized[0] = static_cast<double>(bgr[0] / 255.0);
  bgr_normalized[1] = static_cast<double>(bgr[1] / 255.0);
  bgr_normalized[2] = static_cast<double>(bgr[2] / 255.0);

  return bgr_normalized;
}

bool isInImage(const cv::Mat &image, const cv::Point2d &pixel) {
  return pixel.x >= 0 && pixel.y >= 0 && pixel.x <= image.cols - 1 &&
         pixel.y <= image.rows - 1;
}

ImagePointsd projectPointsToImagePlane(const Points3Dd &points3d,
                                       const cv::Mat &K) {
  ImagePointsd points_projected;
  for (auto const &point3d : points3d) {
    cv::Point2d point_projected = projectPointToImagePlane(point3d, K);
    points_projected.push_back(point_projected);
  }
  return points_projected;
}

Camera::Camera(double fx, double fy, double cx, double cy,
               const vlo::Transform3D &pose)
    : fx_(fx), fy_(fy), cx_(cx), cy_(cy), pose_(pose) {
  pose_inv_ = pose_.inverse();
  K_ = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
}

Camera::Camera(cv::Mat K) {
  fx_ = K.at<double>(0, 0);
  fy_ = K.at<double>(1, 1);
  cx_ = K.at<double>(0, 2);
  cy_ = K.at<double>(1, 2);
  K_ = K;
}

// inline void Camera::setPose(Homo3dTransform pose)
//{
//    pose_ = pose;
//}

// inline Homo3dTransform Camera::getPose(){
//    return pose_;
//}

// inline Homo3dTransform Camera::getCameraMatrixeigen(){

//}

}  // namespace vlo
