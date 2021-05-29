#pragma once
#include <pointmatcher/PointMatcher.h>

#include <memory>

#include "ILidarOdometer.hpp"
#include "common_vlo.hpp"
#include "iostream"
#include "utils_vlo.hpp"

#include <pcl/registration/ndt.h>

using PM = PointMatcher<double>;
using DP = PM::DataPoints;
using Parameters = PM::Parameters;
using TP = PM::TransformationParameters;

/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {

/**
 * @brief The NdtLO class declares the Lidar Odometry based
 * on ICP. The type and all parameters can be defined via config file
 */
class NdtLO : public ILidarOdometer {
 public:
  /**
   * @brief Constructor
   * @param initial pose of NdtLO
   * @param config file with the type of icp point to point, point to plane
   * specified with parameters (e.g. kernels) in config file
   */
  NdtLO(const Transform3D &initial_pose,
                            const std::string &config_file);
  double calculateCrossCheckingError(
      const IFrame::Ptr &current_frame,
      const IFrame::Ptr &reference_frame) override;
  /**
   * @brief A member function to override. Runs the pipeline of lidar
   * odometry.
   * @param lidar_frame Frame/node with acquired point cloud data
   */
  void processFrame(IFrame::Ptr lidar_frame) override;
  void scanToScanTransformICP(const IFrame::Ptr &current_frame,
                              const IFrame::Ptr &reference_frame);
  /**
   * @brief A member function overriden. Computes the transformation between two
   * frames of readings (that are used as nodes)
   * @param current_frame Pointer to the frame with data currently acquired.
   * @param reference_frame Pointer to the frame with acquired in the past time
   * stamp
   * @return It returnes the relative transformation from reference_frame to the
   * current_frame and additionally it informs whether the transformation is
   * reliable (in terms of inner mechanism of failure detection of the
   * respective odometry.
   */
  std::tuple<Transform3D, bool> findFrameToFrameTransformBetween(
      IFrame::Ptr &current_frame, IFrame::Ptr &reference_frame) override;
  std::tuple<Transform3D, bool> findFrameToFrameTransformBetween(
      IFrame::Ptr &current_frame, PointCloud &map) override;
  /**
   * @brief A member function go give hint for icp for frame to frame
   * transformation finding
   * @param Homogenous matrix (translation + rotation)
   */
  void hintForICP(const Transform3D &guess_for_icp) override;
  Eigen::Vector3d scale_ = Eigen::Vector3d::Identity();

 private:
  /**
   * @brief Initial pose [todo: should be moved to IOdometer]
   */
  Transform3D initial_pose_;
  /**
   * @brief A class that is responsible for icp doing
   */
  PM::ICP icp;
  /**
   * @brief Homogenous matrix (translation + rotation)
   */
  Transform3D guess_for_icp_ = Transform3D::Identity();
};

OdometerStruct getNdtLOAndInitialize(Transform3D reference_pose,
                                              std::string config_file,
                                              bool debug, bool verbose);
};  // namespace vlo
