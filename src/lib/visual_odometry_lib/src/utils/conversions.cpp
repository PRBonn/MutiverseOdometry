#include "utils/conversions.hpp"

#include <boost/circular_buffer.hpp>

#include "IFrame.hpp"
#include "visual_odometry/MapPoint.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include "geometry/Camera.hpp"

namespace vlo {

void mapToOpen3D(
    const boost::circular_buffer<vlo::MapPoint::Ptr> &map,
    std::shared_ptr<open3d::geometry::PointCloud> debug_point_cloud) {
  for (const auto &map_point : map) {
    debug_point_cloud->points_.push_back(map_point->position_);
    Eigen::Vector3d color;
    color[0] = map_point->color_[0] / 255.0;
    color[1] = map_point->color_[1] / 255.0;
    color[2] = map_point->color_[2] / 255.0;
    debug_point_cloud->colors_.push_back(Vec3(1.0, 0.0, 0.0));
  }
}

void points3dToOpencv(
    const Points3Dd &map,
    std::shared_ptr<open3d::geometry::PointCloud> debug_point_cloud,
    Vec3 color) {
  for (const auto &map_point : map) {
    Vec3 open3d_point;
    open3d_point[0] = map_point.x;
    open3d_point[1] = map_point.y;
    open3d_point[2] = map_point.z;

    debug_point_cloud->points_.push_back(open3d_point);
    debug_point_cloud->colors_.push_back(color);
  }
};

KeyPoints ImagePointsToKeyPoints(const ImagePointsd &image_points) {
  KeyPoints key_points;
  for (size_t i = 0; i < image_points.size(); i++) {
    key_points.push_back(cv::KeyPoint(image_points[i], 1.f));
  }
  return key_points;
}

KeyPoints ImagePointsToKeyPoints(const ImagePointsd &image_points,
                                 const cv::Mat &mask) {
  KeyPoints key_points;
  for (size_t i = 0; i < image_points.size(); i++) {
    if (!static_cast<bool>(mask.at<uchar>(i))) {
      continue;
    }
    key_points.push_back(cv::KeyPoint(image_points[i], 1.f));
  }
  return key_points;
}

cv::Point3d eigenVectorToPoint3D(const Vec3 &eigen_vector) {
  cv::Point3d point3d;
  point3d.x = eigen_vector[0];
  point3d.y = eigen_vector[1];
  point3d.z = eigen_vector[2];
  return point3d;
}

ImagePointsd keyPointsToImagePoints(const KeyPoints &key_points) {
  ImagePointsd image_points;
  for (const auto &key_point : key_points) {
    image_points.push_back(key_point.pt);
  }

  return image_points;
}

cv::Mat imagePointsToOpenCVMat(const ImagePointsd &image_points) {
  cv::Mat image_points_opencv;

  for (int i = 0; i < image_points.size(); i++) {
    cv::Mat point =
        (cv::Mat_<float>(1, 2) << image_points[i].x, image_points[i].y);
    image_points_opencv.push_back(point);
  }

  return image_points_opencv;
}

bool sortByFirst(const std::pair<double, cv::DMatch> &a,
                 const std::pair<double, cv::DMatch> &b) {
  return (a.first > b.first);
}

bool isInImage(const IFrame::Ptr &current_frame, const cv::Point2d &pixel) {
  return pixel.x > 0 && pixel.y > 0 && pixel.x < current_frame->image_.cols &&
         pixel.y < current_frame->image_.rows;
}



bool isInFrontOfCamera(const Vec3 &pose_querry_point) {
  return pose_querry_point[2] < 0.0;
}

void drawMap(const IFrame::Ptr &current_frame, const cv::Mat &camera_matrix,
             const boost::circular_buffer<MapPoint::Ptr> &map_points,
             const std::string &name_window) {
  if (map_points.size() == 0) {
    return;
  }

  KeyPoints one_occurances;
  KeyPoints two_occurances;
  KeyPoints three_occurances;
  KeyPoints four_occurances;
  KeyPoints more_than_five_occurances;

  Points3Dd points3d;

  for (const auto &map_point : map_points) {
    points3d.push_back(eigenVectorToPoint3D(current_frame->pose_.inverse() *
                                            map_point->position_));
  }

  ImagePointsd points2d = projectPointsToImagePlane(points3d, camera_matrix);

  KeyPoints keypoints = ImagePointsToKeyPoints(points2d);
  for (int i = 0; i < map_points.size(); i++) {
    if (map_points[i]->visible_times_ == 1) {
      one_occurances.push_back(keypoints[i]);
    }
    if (map_points[i]->visible_times_ == 2) {
      two_occurances.push_back(keypoints[i]);
    }
    if (map_points[i]->visible_times_ == 3) {
      three_occurances.push_back(keypoints[i]);
    }
    if (map_points[i]->visible_times_ == 4) {
      four_occurances.push_back(keypoints[i]);
    }
    if (map_points[i]->visible_times_ >= 5) {
      more_than_five_occurances.push_back(keypoints[i]);
    }
  }

  cv::Mat img_out;
  cv::drawKeypoints(current_frame->image_, one_occurances, img_out,
                    cv::Scalar(0, 0, 255));
  cv::imshow(name_window, img_out);
  cv::drawKeypoints(img_out, two_occurances, img_out, cv::Scalar(0, 0, 0));
  cv::imshow(name_window, img_out);
  cv::drawKeypoints(img_out, three_occurances, img_out, cv::Scalar(0, 125, 0));
  cv::imshow(name_window, img_out);
  cv::drawKeypoints(img_out, four_occurances, img_out, cv::Scalar(0, 255, 0));
  cv::imshow(name_window, img_out);
  cv::drawKeypoints(img_out, more_than_five_occurances, img_out,
                    cv::Scalar(255, 0, 0));
  cv::imshow(name_window, img_out);
}

void drawProjectedPoints(const Points3Dd &points_in_current_frame,
                         const IFrame::Ptr &frame_projected_to,
                         const std::string &window_name) {
  ImagePointsd projected_points = projectPointsToImagePlane(
      points_in_current_frame, frame_projected_to->sensor_->getSensorMatrix());
  cv::Mat img;
  cv::drawKeypoints(frame_projected_to->image_,
                    ImagePointsToKeyPoints(projected_points), img,
                    cv::Scalar(0, 255, 0));
  cv::imshow(window_name, img);
}

void drawGoodKeyPoints(const IFrame::Ptr &frame, const std::string &window_name,
                       const double time_stamp, const cv::Scalar color) {
  cv::Mat img;
  cv::drawKeypoints(frame->image_, frame->getGoodKeyPoints(), img, color);
  cv::imshow(window_name + std::to_string(time_stamp), img);
}

void drawKeypoints(const IFrame::Ptr &frame, const std::string &window_name,
                   const double time_stamp, const cv::Scalar color) {
  cv::Mat img;
  cv::drawKeypoints(frame->image_, frame->key_points_, img, color);
  cv::imshow(window_name + std::to_string(time_stamp), img);
}

void drawGoodAndBadKeypoints(const IFrame::Ptr &frame,
                             const std::string &window_name,
                             const double time_stamp) {
  cv::Mat img;
  cv::drawKeypoints(frame->image_, frame->key_points_, img,
                    cv::Scalar(0, 0, 255));
  cv::drawKeypoints(img, frame->getGoodKeyPoints(), img, cv::Scalar(0, 255, 0));
  cv::imshow(window_name + std::to_string(time_stamp), img);
}

void drawPoints(const ImagePointsd &image_points, const IFrame::Ptr &frame,
                const std::string &window_name, const double time_stamp,
                const cv::Scalar color) {
  cv::Mat img;
  cv::drawKeypoints(frame->image_, ImagePointsToKeyPoints(image_points), img,
                    color);
  cv::imshow(window_name + std::to_string(time_stamp), img);
}

void drawMatchesImageToMap(const IFrame::Ptr &frame1,
                           const ImagePointsd &image_points,
                           const IFrame::Ptr &frame2, const Points3Dd &points3d,
                           const std::vector<cv::DMatch> &matches,
                           const std::string &window_name) {
  cv::Mat img_show;

  cv::drawMatches(frame1->image_, ImagePointsToKeyPoints(image_points),
                  frame2->image_,
                  ImagePointsToKeyPoints(projectPointsToImagePlane(
                      points3d, frame1->sensor_->getSensorMatrix())),
                  matches, img_show);
  cv::imshow(window_name, img_show);
}

// void drawMatchesImageToMap(const Frame::Ptr &frame1, const Points3Dd
// &points3d,
//                           const Frame::Ptr &frame2,
//                           const KeyPoints &image_points,
//                           const std::vector<cv::DMatch> &matches,
//                           const std::string &window_name) {
//  cv::Mat img_show;
//  cv::drawMatches(frame1->image_,
//                  ImagePointsToKeyPoints(projectPointsToImagePlane(
//                      points3d, frame1->sensor_->getCameraMatrix())),
//                  frame2->image_, image_points, matches, img_show);
//  cv::imshow(window_name, img_show);
//}

}  // namespace vlo
