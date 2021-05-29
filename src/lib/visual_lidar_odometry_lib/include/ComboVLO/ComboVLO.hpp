#pragma once
#include "common_vlo.hpp"
#include "IFrame.hpp"
#include "IOdometer.hpp"


#include <fstream>
#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "geometry/geometry.hpp"
#include "utils_vlo.hpp"
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {

/**
 * @brief The ComboVLO is composed of all odometries that were developed in this
 * library. In parallel it runs all odometries and choose that give the best
 * estimate (the most optimal one). Currently it uses HuangsVLO, LidarOdometry
 * Point to Point and LidarOdometry Point to Plane.
 */
class ComboVLO : public IOdometer {
public:
    enum ChooseOdometry {FIRST_NOT_FAILED, VISION_LIDAR_ERROR_METRIC, BASED_ON_GT, SANITY_CHECKER_METRIC,ONLY_ONE};
    /**
   * @brief Pointer for an interface.
   */
    using Ptr = std::shared_ptr<ComboVLO>;
    /**
   * @brief Constructor.
   * @param it takes all odometries that should be run in parallel
   * @param initial pose
   */
    ComboVLO(std::vector<OdometerStruct> odometries,
             const Transform3D &initial_pose);
    double calculateCrossCheckingError(const IFrame::Ptr & current_frame, const IFrame::Ptr & reference_frame) override;

    /**
   * @brief A overriden member function. It computes the transformation between
   * two frames of readings (that are used as nodes)
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
        IFrame::Ptr &current_frame, PointCloud &map) override
    {};
    /**
   * @brief Runs the pipeline of visual lidar odometry.
   * @param lidar_frame Frame/node with acquired point cloud data
   */
    void addVisualAndLidarFrames(IFrame::Ptr &current_frame);
    /**
   * @brief update the reference frame and push frame to history [todo: should
   * be moved to Iodometer]
   */
    void addKeyFrame(const IFrame::Ptr &frame);
    /**
   * @brief It push the current transform to the history [todo: should be moved
   * to Iodometer]
   */
    void pushFrameToHistory(const IFrame::Ptr &frame);
    /**
   * @brief Function that based on vector of odometries pick the best choice for
   * estimation
   * @param vector of transformations calculated in parallel, in this function
   * SanityChecker checks the best transformation
   */
    std::pair<Transform3D, bool> chooseOdometry(
        const std::vector<std::pair<Transform3D, bool>>
            &frame_to_frame_transform,  IFrame::Ptr & current_frame,  IFrame::Ptr & reference_frame);
    /**
   * @brief Method name
   */
    std::string method_name = "ComboVLO";

    double calculateErrorToTheReferenceFrameForGivenTransformation(const IFrame::Ptr & current_frame, const IFrame::Ptr & reference_frame, const Transform3D & local_transform);
    /**
   * @brief remove
   */
//    Transform3D camera_lidar_transform_;

    int index_ = 0;
    double acceleration_ = 0.0;
    double velocity = 0.0;
    double ackermann_velocity_ = 0.0;


    Eigen::Matrix4d gt_transform_ = Eigen::Matrix4d::Identity();

    Transform3D frame_to_frame_transform_ = Transform3D::Identity();
  ChooseOdometry choose_odometry_mode_ = ChooseOdometry::ONLY_ONE;
      vlo::Vec3 acceleration_vector_ = vlo::Vec3::Identity();

      PointCloud getLocalMap(const IFrame::Ptr &);
private:
    /**
   * @brief  list of odometries that runs parallel and from which the ComboVLO
   * chooses from
   */
    std::vector<OdometerStruct> odometries_;
//    /**
//   * @brief  current lidar frame with data [todo: should be moved to Iodometer]
//   */
//    IFrame::Ptr current_frame_;
//    /**
//   * @brief  previous lidar frame with data [todo: should be moved to Iodometer]
  // */
  //  IFrame::Ptr reference_frame_;
    /**
   * @brief  state of the odometry [todo: should be moved to Iodometer]
   */
    VLO_STATE state_ = VLO_STATE::INIT;
    /**
   * @brief  initial pose [todo: should be moved to Iodometer]
   */
    Transform3D initial_pose_ = Transform3D::Identity();
    /**
   * @brief history of poses [todo: should be moved to Iodometer]
   */
    std::vector<IFrame::Ptr> trajectory_nodes_;




    std::pair<double, double> evaluateErrorOverlapBetween(const IFrame::Ptr reading, const IFrame::Ptr & reference, const Transform3D & transform);
    std::pair<double, double> evaluateErrorOverlapBetween(const IFrame::Ptr reading, const PointCloud & reference, const Transform3D & transform);
};

}  // namespace vlo
