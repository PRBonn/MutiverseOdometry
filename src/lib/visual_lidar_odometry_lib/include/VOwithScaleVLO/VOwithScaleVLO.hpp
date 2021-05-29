#pragma once
#include <Eigen/Core>
#include <eigen3/Eigen/Core>

#include <opencv2/core/eigen.hpp>
#include <opencv2/xfeatures2d/cuda.hpp>

#include "IFrame.hpp"
#include "IVisualOdometer.hpp"
#include "SanityChecker/SanityChecker.hpp"
#include "common_vlo.hpp"
#include "visual_odometry/FeatureDetector.hpp"
#include "visual_odometry/FeatureMatcher.hpp"
#include "visual_odometry/Map.hpp"
#include "visual_odometry/MapPoint.hpp"
#include "visual_odometry/VisualFrame.hpp"
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {
/**
 * @brief The VOwithScaleVLO class declares the Visual Odometry based on
 * 5 point algorithm.
 */
class VOwithScaleVLO : public IVisualOdometer {
public:
    /**
   * @brief Pointer for an interface.
   */
    using Ptr = std::shared_ptr<VOwithScaleVLO>;
    /**
   * @brief Constructor
   * @param feature_detector Descriptor that is used for image [todo: remove]
   * @param matcher Matcher type used
   * @param initial_pose initial pose
   */
    VOwithScaleVLO(const cv::Ptr<cv::Feature2D> &feature_detector,
                           const cv::Ptr<cv::DescriptorMatcher> &matcher,
                           const Transform3D &initial_pose);
     double calculateCrossCheckingError(const IFrame::Ptr & current_frame, const IFrame::Ptr & reference_frame) override;
    /**
   * @brief Default destructor.
   */
    ~VOwithScaleVLO() = default;
    /**
   * @brief A member function to override. Runs the pipeline of lidar
   * odometry.
   * @param visual_frame Frame/node with acquired point cloud data
   */
    void processFrame(IFrame::Ptr &frame) override;
    /**
   * @brief update the reference frame and push frame to history [todo: should
   * be moved to Iodometer]
   */
    void addKeyFrame(IFrame::Ptr);
    /**
   * @brief function that takes externally scale e.g. from the ground truth
   */
    void calculateScale(const Vec3 &) override;
    /**
   * @brief Calculate transform based on 5point algorithm
   * @param current frame
   * @param reference frame
   */
    std::tuple<Transform3D, bool> imageToimageTransform5point(
        IFrame::Ptr &current_frame, IFrame::Ptr &reference_frame);
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
        IFrame::Ptr &current_fraem, IFrame::Ptr &reference_frame) override;
    std::tuple<Transform3D, bool> findFrameToFrameTransformBetween(
        IFrame::Ptr &current_frame, PointCloud &map) override
    {};
    /**
   * @brief Calculate matches between the current frame and reference frame
   * @param current frame
   * @param reference frame
   */
    void findCurrentImageToReferenceImageMatches(IFrame::Ptr &current_frame,
                                                 IFrame::Ptr &reference_frame);
    std::string method_name = "vo_only";

private:
    /**
   * @brief Feature matcher that Visual Odometry uses.
   */
    FeatureMatcher feature_matcher_;
    /**
   * @brief Feature detector that Visual Odometry uses [remove]
   */
    FeatureDetector feature_detector_;

};
}  // namespace vlo
