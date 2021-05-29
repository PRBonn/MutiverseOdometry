#include "SanityChecker/LineDetector.hpp"

#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
namespace vlo {

LineDetector::LineDetector() {}

cv::Mat ExtractChannel(cv::Mat &image, int colorspace, int lowerBound,
                       int upperBound, int channel = 0) {
  cv::Mat colorspace_img;
  cv::cvtColor(image, colorspace_img, colorspace);
  cv::imshow("colorspace_img", colorspace_img);
  cv::Mat channels[3];
  cv::split(colorspace_img, channels);
  auto channel_img = channels[channel];
  cv::Mat binary_img;
  cv::inRange(channel_img, lowerBound, upperBound, binary_img);
  return binary_img;
}

cv::Mat Sobel(cv::Mat wrapped_img, int lowerBound, int upperBound, std::string sobel_type,
              int kernelSize = 3) {
  cv::Mat gray_img, sobelx_img, sobely_img;
  cv::cvtColor(wrapped_img, gray_img, cv::COLOR_RGB2GRAY);
  cv::Sobel(gray_img, sobelx_img, 1, 1, 0, kernelSize);
  cv::Sobel(gray_img, sobely_img, 1, 0, 1, kernelSize);
  auto abs_sobelx = cv::abs(sobelx_img);
  auto abs_sobely = cv::abs(sobely_img);
  cv::Mat abs_sobelx_square, abs_sobely_square;
  cv::pow(abs_sobelx, 2, abs_sobelx_square);
  cv::pow(abs_sobely, 2, abs_sobely_square);
  cv::Mat grad;
  cv::sqrt(abs_sobelx_square + abs_sobely_square, grad);
  cv::Mat orientation;  //= std::atan2(abs_sobelx, abs_sobely);
  cv::Mat valParam;

  if (sobel_type =="x") {
    valParam = abs_sobelx;
  } else if (sobel_type == "y") {
    valParam = abs_sobely;
  } else if (sobel_type == "xy") {
    valParam = grad;
  } else {
    valParam = orientation;
  }

  cv::Mat binary_img;
    cv::inRange(binary_img, lowerBound, upperBound, binary_img);
}

void LineDetector::findLines(const IFrame::Ptr &frame) {
  cv::Mat img = imread(
      "/home/andrzej/git/Udacity-CarND-Advanced-Lane-Lines/test_images/"
      "straight_lines1.jpg",
      cv::IMREAD_COLOR);
  cv::Point2f src_left_down(150, 720);
  cv::Point2f src_right_down(1250, 720);
  cv::Point2f src_right_upper(700, 450);
  cv::Point2f src_left_upper(590, 450);
  std::vector<cv::Point2f> src_vertices;
  src_vertices.push_back(src_left_down);
  src_vertices.push_back(src_left_upper);
  src_vertices.push_back(src_right_upper);
  src_vertices.push_back(src_right_down);

  cv::Point2i int_src_left_down(150, 720);
  cv::Point2i int_src_right_down(1250, 720);
  cv::Point2i int_src_right_upper(700, 450);
  cv::Point2i int_src_left_upper(590, 450);
  std::vector<cv::Point2i> int_src_vertices;
  int_src_vertices.push_back(int_src_left_down);
  int_src_vertices.push_back(int_src_right_down);
  int_src_vertices.push_back(int_src_right_upper);
  int_src_vertices.push_back(int_src_left_upper);

  cv::Point2f tgt_left_down(200, 720);
  cv::Point2f tgt_right_down(200, 0);
  cv::Point2f tgt_right_upper(980, 0);
  cv::Point2f tgt_left_upper(980, 720);

  std::vector<cv::Point2f> tgt_vertices;

  tgt_vertices.push_back(tgt_left_down);
  tgt_vertices.push_back(tgt_right_down);
  tgt_vertices.push_back(tgt_right_upper);
  tgt_vertices.push_back(tgt_left_upper);
  cv::Mat nice_img;
  img.copyTo(nice_img);
  cv::polylines(nice_img, int_src_vertices, true, cv::Scalar(0, 0, 255), 10);
  cv::imshow("img", img);

  std::cout << tgt_vertices.size() << std::endl;

  std::cout << src_vertices.size() << std::endl;

  auto M = cv::getPerspectiveTransform(src_vertices, tgt_vertices);
  cv::Mat wrapped_img;
  cv::warpPerspective(img, wrapped_img, M, img.size());
  cv::imshow("wrapped img", wrapped_img);

  auto lower_bound_threshold = 150;
  auto upper_bound_threshold = 255;
  auto sobel_threshold_min = 20;
  auto sobel_threshold_max = 100;

  cv::Mat s_channel =
      ExtractChannel(wrapped_img, cv::COLOR_RGB2HLS, lower_bound_threshold,
                     upper_bound_threshold, 2);
  cv::Mat l_channel =
      ExtractChannel(wrapped_img, cv::COLOR_RGB2HLS, lower_bound_threshold,
                     upper_bound_threshold, 1);
  cv::Mat y_channel =
      ExtractChannel(wrapped_img, cv::COLOR_RGB2YUV, lower_bound_threshold,
                     upper_bound_threshold, 0);

  cv::imshow("s_channel", s_channel);
  cv::imshow("l_chanel", l_channel);
  cv::imshow("y_channel", y_channel);
  cv::imshow("wrapped_img", wrapped_img);
  cv::Mat sobelx_img = Sobel(wrapped_img,200,100,"x");
  cv::imshow("sobel x img", sobelx_img);
  cv::waitKey(0);

  //  cv::Mat sobeldir;
  //  cv::Sobel(wrapped_img, sobeldir, sobelx_img.type(), 1, 1, 0.3, 50);
  //  cv::Mat out_channel = (s_channel & l_channel & y_channel) | sobelx_img |
  //  sobeldir;
  ////  cv::bitwise_and(s_channel, l_channel,out_channel);
  ////cv::bitwise_and(out_channel, y_channel,out_channel);
  ////cv::Mat output_sobel;
  ////cv::bitwise_or(sobelx_img, sobeldir, output_sobel);

  ////cv::Mat output_finally;
  ////cv::bitwise_and(output_sobel, out_channel, output_finally);

  ////auto lol = s_channel & l_channel;

  //  cv::Mat new_img = cv::Mat::zeros(sobeldir.rows, sobeldir.cols, CV_64FC1);
  //  cv::waitKey(0);

  //  exit(0);

  //  frame->point_cloud_.EstimateNormals();
  //  auto [reference_points, reference_indexes] =
  //      frame->point_cloud_.SegmentPlane(0.1);
  //  auto plane = frame->point_cloud_.SelectByIndex(reference_indexes, false);
  //  std::cout << " plane has: " << plane->points_.size() << std::endl;
  //  auto line_candidates = std::make_shared<PointCloud>();
  //  //  std::vector<cv::Point2d> projected3dpointsToImagePlane;
  //  //  std::vector<cv::KeyPoint> keypoints_heeh;
  //  //  for (size_t i = 0; i < plane->points_.size(); i++) {
  //  //    if (plane->colors_[i][0] > 0.45f and plane->points_[i][0] > 0.0f) {
  //  //      line_candidates->points_.push_back(plane->points_[i]);
  //  //      line_candidates->colors_.push_back(plane->colors_[i]);
  //  //      cv::KeyPoint keypoint;
  //  //      auto lol = keypoint.pt = projectPointToImagePlane(
  //  //          camera_lidar_transform_.inverse() * plane->points_[i],
  //  //          frame->sensor_->getSensorMatrix());
  //  //      keypoints_heeh.push_back(keypoint);
  //  //    }
  //  //  }
  //  cv::Mat_<cv::Point2f> projected_points3d_mat;
  //  std::vector<cv::KeyPoint> projected_points3d_mat_keypoints;
  //  for (size_t i = 0; i < plane->points_.size(); i++) {
  //    if (/*plane->colors_[i][0] >= 0.5f and */ std::abs(plane->points_[i][1])
  //    <
  //            5.0 and
  //        plane->points_[i][0] > 0.0f) {
  //      auto point_camera_frame = camera_lidar_transform_ * plane->points_[i];
  //      auto projected_point_camera_frame = projectPointToImagePlane(
  //          point_camera_frame, frame->sensor_->getSensorMatrix());

  //      if (isInImage(frame->image_, projected_point_camera_frame) and
  //          point_camera_frame[2] > 0.0) {
  //        projected_points3d_mat.push_back(
  //            cv::Mat(cv::Point2f(projected_point_camera_frame)).t());
  //        cv::KeyPoint point;
  //        point.pt = cv::Point2f(projected_point_camera_frame);
  //        projected_points3d_mat_keypoints.push_back(point);
  //        line_candidates->points_.push_back(plane->points_[i]);
  //      }
  //    }
  //  }

  //  auto convexpoints = findTrapeoisodalConvexHull(
  //      frame->image_, projected_points3d_mat_keypoints);
  //  std::vector<cv::Point2d> correspondedpoint;

  //  std::vector<cv::Point2d> src;

  //  src.push_back(cv::Point2d(500,170));
  //  src.push_back(cv::Point2d(700,170));
  //  src.push_back(cv::Point2d(1200,290));
  //  src.push_back(cv::Point2d(200,290));

  //  auto  offset=100;
  //  auto height = frame->image_.rows;
  //  auto width=  frame->image_.cols;
  //    std::vector<cv::Point2d> dst;
  //  dst.push_back(cv::Point2d(offset,0));
  //  dst.push_back(cv::Point2d(width-offset,0));
  //  dst.push_back(cv::Point2d(width-offset,height));
  //  dst.push_back(cv::Point2d(offset,height));

  //  auto H = cv::findHomography(src, dst);
  //  cv::Mat img1_warp;
  //  cv::warpPerspective(frame->image_, img1_warp, H, frame->image_.size());
  //  cv::imshow("LOL",img1_warp);

  //  cv::Mat img_out;
  //  cv::drawKeypoints(frame->image_, projected_points3d_mat_keypoints,
  //  img_out,
  //                    cv::Scalar(0, 0, 255));
  //  int color = 50;
  //  for (const auto &point : convexpoints) {
  //    std::cout << point << std::endl;
  //    cv::circle(img_out, point, 10, cv::Scalar(color, 0, 0), 5);
  //    color = color + 50;
  //  }
  //  cv::imshow("window_name", img_out);
  //  cv::waitKey(1);

  //  lines_ = line_candidates;
  //}

  // std::vector<cv::Point2d> LineDetector::findTrapeoisodalConvexHull(
  //    const cv::Mat &image, const std::vector<cv::KeyPoint> &keypoints) {
  //  std::cout << "looking for points " << std::endl;
  //  cv::Point2d left_bottom_corner(image.cols / 2, 3 * image.rows / 4);
  //  cv::Point2d left_top_corner(image.cols / 2, 3 * image.rows / 4);
  //  cv::Point2d right_bottom_corner(image.cols / 2, 3 * image.rows / 4);
  //  cv::Point2d right_top_corner(image.cols / 2, 3 * image.rows / 4);

  //  std::vector<cv::Point2d> corner_points;
  //  for (auto &keypoint : keypoints) {
  //    if (keypoint.pt.x < left_bottom_corner.x) {
  //      left_bottom_corner = keypoint.pt;
  //    }
  //    if (keypoint.pt.x > right_bottom_corner.x) {
  //      right_bottom_corner = keypoint.pt;
  //    }

  //    if (keypoint.pt.x > right_top_corner.x) {
  //      if (keypoint.pt.y < right_top_corner.y + 0.5) {
  //        right_top_corner = keypoint.pt;
  //      }
  //    }
  //    if (keypoint.pt.x < left_top_corner.x) {
  //      if (keypoint.pt.y < left_top_corner.y + 0.5) {
  //        left_top_corner = keypoint.pt;
  //      }
  //    }
  //  }

  //  corner_points.push_back(left_top_corner);
  //  corner_points.push_back(left_bottom_corner);
  //  corner_points.push_back(right_bottom_corner);
  //  corner_points.push_back(right_top_corner);
  //  return corner_points;
}
}  // namespace vlo
