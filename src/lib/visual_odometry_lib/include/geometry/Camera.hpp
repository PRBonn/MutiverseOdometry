#pragma once
#include <opencv2/core/core.hpp>

#include "ISensor.hpp"
#include "common_vlo.hpp"
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {
using ImagePointsd = std::vector<cv::Point2d>;
using Points3Dd = std::vector<cv::Point3d>;

// cv::Point2f projectPointToImagePlane(const cv::Mat &p, const cv::Mat &K);
/**
 * @brief Project 3D point to the image plane
 * @param 3D points
 * @param Intrinsic camera matrix.
 */
cv::Point2d projectPointToImagePlane(const cv::Point3d &p, const cv::Mat &K);
/**
 * @brief Project 3D point to the image plane
 * @param 3D points
 * @param Intrinsic camera matrix.
 */
cv::Point2d projectPointToImagePlane(const Vec3 &, const cv::Mat &K);
/**
 * @brief Project 3D points to the image plane
 * @param 3D points
 * @param Intrinsic camera matrix.
 */
std::vector<cv::Point2d> projectPointsToImagePlane(const Points3Dd &points3d,
                                                   const cv::Mat &K);
/**
 * @brief Gets RGB values from the image for given point
 * @param image
 * @param image point
 */
Eigen::Vector3d getRGBvalueFromImageNormalized(const cv::Mat &img,
                                               const cv::Point2d &image_point);

/**
 * @brief Transform point 3D to camera frame.
 * @param point 3d.
 * @param Transformation 3D points to the camera reference frame.
 */
cv::Mat world2camera(const cv::Point3f &p, const cv::Mat &T_world_to_cam);
/**
 * @brief It checks whether pixel is in boundaries of image boundaries (e.g.
 * after projection)
 * @param image
 * @param point2d (can be in subpixel acc)
 */
bool isInImage(const cv::Mat &image, const cv::Point2d &pixel);
/**
 * @brief  The Camera class declares a functionality for a frame for a
 * camera-type of sensor.
 */
class Camera : public ISensor {
 public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  /**
   * @brief Pointer for an interface.
   */
  using Ptr = std::shared_ptr<Camera>;
  /**
   * @brief Constructor default
   */
  Camera();
  /**
   * @brief Constructor
   * @param focal length in x
   * @param focal length in y
   * @param principal point offset x
   * @param principal point offset y
   * @param pose of a camera in base link frame
   */
  Camera(double fx, double fy, double cx, double cy,
         const vlo::Transform3D &pose);
  /**
   * @brief Constructor
   * @param intrinsic camera matrix
   */
  Camera(cv::Mat K);
  /**
   * @brief Get Camera intrinsic matrix
   */
  inline cv::Mat K() const { return K_; }
  /**
   * @brief Get Camera intrinsic matrix
   */
  inline cv::Mat getSensorMatrix() const override { return K_; }

 private:
  /**
   * @brief Focal length in x.
   */
  double fx_ = 0;
  /**
   * @brief Focal length in y.
   */
  double fy_ = 0;
  /**
   * @brief Principal point offset x
   */
  double cx_ = 0;
  /**
   * @brief Principal point offset y
   */
  double cy_ = 0;
  /**
   * @brief Pose of the camera in the base link frame.
   */
  vlo::Transform3D pose_;
  /**
   * @brief Inverse pose of the camera in the base link frame.
   */
  vlo::Transform3D pose_inv_;
  /**
   * @brief Camera intrinsic matrix.
   */
  cv::Mat K_;
};
}  // namespace vlo
