#include "VisualLidarFrame.hpp"

namespace vlo {

VisualLidarFrame::VisualLidarFrame(const VisualFrame::Ptr &visual_frame,
                                   const LidarFrame::Ptr &lidar_frame, const Transform3D &camera_lidar_transform, bool color ) {
    visual_frame_ = visual_frame;
    lidar_frame_ = lidar_frame;

  image_ = visual_frame->image_;
  key_points_ = visual_frame->key_points_;
  descriptors_ = visual_frame->descriptors_;
  good_indexes_ = visual_frame->good_indexes_;
  id_ = visual_frame->id_;
  sensor_ = visual_frame->sensor_;
  camera_lidar_transform_ = camera_lidar_transform;
  time_stamp_ = visual_frame->time_stamp_;


  if (color)
  {
       point_cloud_ = lidar_frame->point_cloud_;
      std::cout << "Colored activation." << std::endl;
      colored = color;

      for (size_t i = 0 ; i < lidar_frame->point_cloud_.points_.size() ; i ++)
      {
    cv::Point2d projected_point_camera_frame = projectPointToImagePlane(
              lidar_frame->point_cloud_.points_[i], visual_frame->sensor_->getSensorMatrix());

    if (isInImage(image_, projected_point_camera_frame) and
        lidar_frame->point_cloud_.points_[i][2] > 0.0) {
       auto img_color = getRGBvalueFromImageNormalized(image_, projected_point_camera_frame);
        point_cloud_colored_.points_.push_back(lidar_frame->point_cloud_.points_[i]);
       point_cloud_colored_.colors_.push_back(
            Vec3(img_color[2], img_color[1], img_color[0]));
       point_cloud_colored_.normals_.push_back(lidar_frame->point_cloud_.normals_[i]);
    }
      }
  } else {
      std::cout << "Color NOT activated." << std::endl;
       point_cloud_ = lidar_frame->point_cloud_;
  }


}

void VisualLidarFrame::updateGoodIndexes(Indexes &indexes) {
  good_indexes_ = indexes;
};

ImagePointsd VisualLidarFrame::getGoodPoints() const {
  ImagePointsd good_points;

  for (const auto &good_index : good_indexes_) {
    good_points.push_back(key_points_[good_index].pt);
  }

  return good_points;
}
KeyPoints VisualLidarFrame::getGoodKeyPoints() const {
  KeyPoints key_points;
  for (const auto &good_index : good_indexes_) {
    key_points.push_back(key_points_[good_index]);
  }

  return key_points;
}

cv::Mat VisualLidarFrame::getGoodDescriptors() const {
    cv::Mat descriptors;
    for (const auto &good_index : good_indexes_) {
        descriptors.push_back(descriptors_.row(good_index));
    }

    return descriptors;
}
double VisualLidarFrame::evaluateOverlappingWithFrame(const IFrame::Ptr & frame, const Transform3D & transform_from_this_to_frame)
{
    double lidar_part = lidar_frame_->evaluateOverlappingWithFrame(frame, transform_from_this_to_frame);
    std::cout << "lidar part: " << lidar_part << std::endl;

}
}  // namespace vlo
