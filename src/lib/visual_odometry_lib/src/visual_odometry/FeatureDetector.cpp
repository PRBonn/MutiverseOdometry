#include "visual_odometry/FeatureDetector.hpp"

namespace vlo {


FeatureDetector::FeatureDetector(cv::Ptr<cv::Feature2D> descriptor) {
  descriptor_ = descriptor;
}



void FeatureDetector::calculateKeyPoints(IFrame::Ptr &frame) {
  descriptor_->detect(frame->image_, frame->key_points_);
}

void FeatureDetector::calculateDescriptors(IFrame::Ptr &frame) {
  descriptor_->compute(frame->image_, frame->key_points_, frame->descriptors_);
}

void FeatureDetector::calculateKeyPoints(const cv::Mat &image,
                                         KeyPoints &key_points) {
  descriptor_->detect(image, key_points);
}

void FeatureDetector::calculateDescriptors(const cv::Mat &image,
                                           KeyPoints &key_points,
                                           cv::Mat &descriptors) {
  descriptor_->compute(image, key_points, descriptors);
}

void FeatureDetector::calculateKeyPointsAndDescriptors(IFrame::Ptr &frame) {
  calculateKeyPoints(frame);
  calculateDescriptors(frame);
}

void FeatureDetector::calculateKeyPointsAndDescriptors(
    const cv::Mat &image, KeyPoints &out_key_points, cv::Mat &out_descriptors) {
  calculateKeyPoints(image, out_key_points);
  calculateDescriptors(image, out_key_points, out_descriptors);
}

void FeatureDetector::calculateGridKeyPointsAndDescriptors(
    const cv::Mat &image, int grid_size, int max_number_key_points_per_cell, KeyPoints & keypoints, cv::Mat &descriptors) {
  calculateKeyPoints(image, keypoints);

  int rows_in_grid = image.rows / grid_size;
  int cols_in_grid = image.cols / grid_size;

  Eigen::MatrixXi grid = Eigen::MatrixXi::Zero(grid_size, grid_size);

  KeyPoints temp_keypoints;

//  std::cout << "keypoints: " << frame->key_points_.size() << std::endl;
//  std::cout << "descriptors: " << frame->descriptors_.size() << std::endl;
//  std::cout << "Key Points " << frame->key_points_.size() << std::endl;
  for (auto &key_point : keypoints) {
    int row = (static_cast<int>(key_point.pt.y)) / rows_in_grid;
    if (row >= grid_size) {
      row = grid_size - 1;
    }
    int col = (static_cast<int>(key_point.pt.x)) / cols_in_grid;
    if (col >= grid_size) {
      col = grid_size - 1;
    }
    if (grid(row, col) < max_number_key_points_per_cell) {
      temp_keypoints.push_back(key_point);
      grid(row, col)++;

      if (grid(row, col) > max_number_key_points_per_cell) break;
    }
  }
  std::cout << grid << std::endl;

  keypoints = temp_keypoints;
  calculateDescriptors(image, keypoints, descriptors);

//  std::cout << "keypoints: " << frame->key_points_.size() << std::endl;
//  std::cout << "descriptors: " << frame->descriptors_.size() << std::endl;
}

/* //  std::vector<std::vector<KeyPoints>> grid_with_elements(
  //      grid_size, std::vector<KeyPoints>(grid_size));
 *
 *
  //  std::cout << "Key Points " << frame->key_points_.size() << std::endl;
  //  for (const auto &key_point : frame->key_points_) {
  //    int row = (static_cast<int>(key_point.pt.y)) / rows_in_grid;
  //    if (row >= grid_size) {
  //      row = grid_size - 1;
  //    }
  //    int col = (static_cast<int>(key_point.pt.x)) / cols_in_grid;
  //    if (col >= grid_size) {
  //      col = grid_size - 1;
  //    }
  //    grid_with_elements[row][col].push_back(key_point);
  //  }

  //  for (auto i = 0; i < grid_with_elements.size(); i++) {
  //    for (auto j = 0; j < grid_with_elements[i].size(); j++) {
  //      std::cout << " i " << i << std::endl;
  //      std::cout << "j" << j << std::endl;
  //      std::cout << grid_with_elements[i][j].size() << " ";
  //      if (grid_with_elements[i][j].size() != 0) {
  //        KeyPoints out;
  //        std::sample(grid_with_elements[i][j].begin(),
  //                    grid_with_elements[i][j].end(), std::back_inserter(out),
  //                    max_number_key_points_per_cell,
  //                    std::mt19937{std::random_device{}()});
  //        temp_keypoints.insert(temp_keypoints.end(), out.begin(), out.end());
  //        std::cout << " lol " << std::endl;

  //        std::cout << "temp keys " << temp_keypoints.size() << std::endl;
  //      } else {
  //        std::cout << "Skipped!" << std::endl;
  //      }
  //    }
  //    frame->key_points_ = temp_keypoints;
  //    calculateDescriptors(frame);

  //    std::cout << "keypoints: " << frame->key_points_.size() << std::endl;
  //    std::cout << "descriptors: " << frame->descriptors_.size() << std::endl;
  //  }
//  //}
//  std::cout << grid << std::endl;

//  frame->key_points_ = temp_keypoints;
//  calculateDescriptors(frame);

**/
};  // namespace vlo
