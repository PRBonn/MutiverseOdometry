#pragma once
#include <open3d/Open3D.h>

#include <boost/circular_buffer.hpp>
#include <opencv2/core.hpp>

#include "IFrame.hpp"
#include "common_vlo.hpp"
#include "visual_odometry/MapPoint.hpp"
#include "geometry/Camera.hpp"
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {
/**
 * @brief Function convert map kept as a circular buffor to Open3d.
 * @param 3D map
 * @param open3d point cloud
 */
void mapToOpen3D(
    const boost::circular_buffer<vlo::MapPoint::Ptr> &map,
    std::shared_ptr<open3d::geometry::PointCloud> debug_point_cloud);
/**
 * @brief Function convert opencv point3D to Open3d.
 * @param 3D points
 * @param open3d point cloud
 */
void points3dToOpencv(
    const Points3Dd &map,
    std::shared_ptr<open3d::geometry::PointCloud> debug_point_cloud,
    Vec3 color = Vec3(1.0, 0.0, 0.0));
/**
 * @brief Converts vector of points2d to keypoints
 * @param vector of points2d
 * @param vector of keypoints
 */
KeyPoints ImagePointsToKeyPoints(const ImagePointsd &image_points);
/**
 * @brief Converts vector of points2d to keypoints with mask
 * @param vector of points2d
 * @param vector of mask
 * @return vector of keypoints
 */
KeyPoints ImagePointsToKeyPoints(const ImagePointsd &image_points,
                                 const cv::Mat &mask);
/**
 * @brief Converts Eigen::Vector3 vector to cv::Point3D
 * @param eigen::vector 3
 * @return cv::Point3d
 */
cv::Point3d eigenVectorToPoint3D(const Vec3 &vector);
/**
 * @brief Converts Eigen::Vector3 vector to cv::Point3D
 * @param eigen::vector 3
 * @return cv::Point3d
 */
ImagePointsd keyPointsToImagePoints(const KeyPoints &key_points);
/**
 * @brief Converts Eigen::Vector3 vector to cv::Point3D
 * @param eigen::vector 3
 * @return cv::Point3d
 */
cv::Mat imagePointsToOpenCVMat(const ImagePointsd &image_points);
/**
 * @brief Compare pairs of (double, cv::DMatch) to order matches with the least
 * error value
 */
bool sortByFirst(const std::pair<double, cv::DMatch> &a,
                 const std::pair<double, cv::DMatch> &b);
/**
 * @brief Draw Maps that is currently storated in the map
 * @param current frame
 * @param intrinsic camera matrix
 * @param map points
 * @param window name
 */
void drawMap(const IFrame::Ptr &current_frame, const cv::Mat &camera_matrix,
             const boost::circular_buffer<MapPoint::Ptr> &map_points,
             const std::string &window_name = "Map");
/**
 * @brief Check whether point 2d (i.e. projected) is in the field of view of the
 * image
 * @param current frame with the image
 * @param pixel index
 */
bool isInImage(const IFrame::Ptr &current_frame, const cv::Point2d &pixel);
/**
 * @brief Check whether point3D is in fron or behind camera.
 * @param querry points coordinates
 */
bool isInFrontOfCamera(const Vec3 &pose_querry_point);
/**
 * @brief Draw 3D points projected to the 2D plane.
 * @param Points 3D.
 * @param Frame from where to draw points on the image plane.
 * @param Window name
 */
void drawProjectedPoints(const Points3Dd &points, const IFrame::Ptr &frame,
                         const std::string &window_name);
/**
 * @brief It draws KeyPoints that are currently in the frame/node
 * @param frame from where draw points
 * @param window name
 * @param time stamp
 * @param color of points
 */
void drawKeypoints(const IFrame::Ptr &frame, const std::string &window_name,
                   const double time_stamp, const cv::Scalar color);
/**
 * @brief It draws Points that are currently in the frame/node but only with
 * good indexes
 * @param frame from where draw points
 * @param window name
 * @param time stamp
 * @param color of points
 */
void drawGoodKeyPoints(const IFrame::Ptr &frame, const std::string &window_name,
                       const double time_stamp, const cv::Scalar color);
/**
 * @brief It draws Points that are currently in the frame/node with different
 * colors
 * @param frame from draw points
 * @param window name
 * @param time stamp
 */
void drawGoodAndBadKeypoints(const IFrame::Ptr &frame,
                             const std::string &window_name,
                             const double time_stamp);
/**
 * @brief It draws Points that are currently in the frame/node
 * @param points that are to drawn
 * @param frame from where draw points
 * @param window name
 * @param time stamp
 * @param color of points
 */
void drawPoints(const ImagePointsd &image_points, const IFrame::Ptr &frame,
                const std::string &window_name, const double time_stamp,
                const cv::Scalar color = cv::Scalar(0, 0, 255));
/**
 * @brief Draw matches from 2D Image to Points3d
 * @param frame 1
 * @param points 3d associated with frame 1
 * @param frame 2
 * @param image associated with frame 2
 * @param matches
 * @param window name
 */
void drawMatchesImageToMap(const IFrame::Ptr &frame1, const Points3Dd &points3d,
                           const IFrame::Ptr &frame2,
                           const ImagePointsd &image_points,
                           const std::vector<cv::DMatch> &matches,
                           const std::string &window_name);
/**
 * @brief Draw matches from 3D to Image
 * @param frame 1
 * @param points 3d associated with frame 1
 * @param frame 2
 * @param image associated with frame 2
 * @param matches
 * @param window name
 */
void drawMatchesImageToMap(const IFrame::Ptr &frame1,
                           const ImagePointsd &image_points,
                           const IFrame::Ptr &frame2, const Points3Dd &points3d,
                           const std::vector<cv::DMatch> &matches,
                           const std::string &window_name);
}  // namespace vlo
