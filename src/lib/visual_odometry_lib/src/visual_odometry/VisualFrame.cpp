#include "visual_odometry/VisualFrame.hpp"

#include "visual_odometry/FeatureDetector.hpp"
namespace vlo {
int VisualFrame::factory_id_ = 0;

VisualFrame::VisualFrame(const cv::Mat &image, const Camera::Ptr &camera,
                         const double &time_stamp) {
  image_ = image;
  id_ = factory_id_++;
  time_stamp_ = time_stamp;
  sensor_ = camera;
//  feature_detector.calculateKeyPointsAndDescriptors(image, key_points_,
//                                                    descriptors_);
//  feature_detector.calculateGridKeyPointsAndDescriptors(image, 50,15, key_points_, descriptors_);
}

Mat34 VisualFrame::getProjectionMatrix_eigen() const {
  Mat33 camera_matrix;
  cv::cv2eigen(sensor_->getSensorMatrix(), camera_matrix);
  Mat34 projection_matrix = camera_matrix * pose_.matrix().block<3, 4>(0, 0);
  // Mat34 projection_matrix =
  // camera_to_world_transform_.matrix().block<3,4>(0,0);
  return projection_matrix;
}

cv::Mat VisualFrame::getProjectionMatrix() const {
  auto projection_matrix_eigen = getProjectionMatrix_eigen();
  cv::Mat projection_matrix_opencv;
  cv::eigen2cv(projection_matrix_eigen, projection_matrix_opencv);
  return projection_matrix_opencv;
}

void VisualFrame::addMapPoint(const MapPoint::Ptr &point) {
  points3d_.insert(point);
}

cv::Mat VisualFrame::getGoodDescriptorFromIndex(int index) {
  auto good_descriptor_index = this->good_indexes_[index];
  return descriptors_.row(good_descriptor_index);
}

cv::KeyPoint VisualFrame::getGoodKeyFromIndex(int index) {
  return key_points_[index];
}

KeyPoints VisualFrame::getGoodKeyPoints() const {
  KeyPoints key_points;

  for (const auto &good_index : good_indexes_) {
    key_points.push_back(key_points_[good_index]);
  }

  return key_points;
}

ImagePointsd VisualFrame::getGoodPoints() const {
  ImagePointsd good_points;

  for (const auto &good_index : good_indexes_) {
    good_points.push_back(key_points_[good_index].pt);
  }

  return good_points;
}


ImagePointsd VisualFrame::getPoints() const {
  ImagePointsd good_points;

  for (const auto &key_point : key_points_) {
    good_points.push_back(key_point.pt);
  }

  return good_points;
}

cv::Mat VisualFrame::getGoodDescriptors() const {
  cv::Mat descriptors;

  for (const auto &good_index : good_indexes_) {
    descriptors.push_back(descriptors_.row(good_index));
  }
  return descriptors;
}

// if another mask is coming for descriptors we need to cover it by calling this
// function
void VisualFrame::updateGoodIndexes(Indexes &indexes) {
  good_indexes_ = indexes;
}

void VisualFrame::printIndexes() const {
  for (const auto &good_index : good_indexes_) {
    std::cout << good_index << std::endl;
  }
}
double VisualFrame::evaluateOverlappingWithFrame(const IFrame::Ptr & frame, const Transform3D & transform_from_this_to_frame) {



}

}  // namespace vlo
