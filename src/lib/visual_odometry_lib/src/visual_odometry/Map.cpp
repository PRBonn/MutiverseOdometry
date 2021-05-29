#include "visual_odometry/Map.hpp"


namespace vlo {

Map::Map() { map_points_.set_capacity(50000); }

void Map::insertKeyFrame(VisualFrame::Ptr frame) {
  if (keyframes_.find(frame->id_) == keyframes_.end()) {
    keyframes_.insert(make_pair(frame->id_, frame));
  } else {
    keyframes_[frame->id_] = frame;
  }
}

int Map::size() const { return map_points_.size(); }
MapPoint::Ptr Map::operator[](int index) { return map_points_[index]; }
cv::Mat Map::getDescriptors() const {
  cv::Mat descriptors;
  for (const auto& map_point : map_points_) {
    descriptors.push_back(map_point->descriptor_);
  }
  return descriptors;
}

void Map::addMapPoint(const MapPoint::Ptr &point) { map_points_.push_back(point); }

boost::circular_buffer<vlo::MapPoint::Ptr> Map::getMap() { return map_points_; }

void Map::eraseMap() { map_points_.clear(); }

void Map::getPointsExpressedInGivenFrame(const VisualFrame::Ptr &frame,  Points3Dd & points3d)
{
    for (const auto &map_point : map_points_) {
      cv::Point3d map_point3d = eigenVectorToPoint3D(
          frame->pose_.inverse() * map_point->position_);
      points3d.push_back(map_point3d);
    }
}



}  // namespace vlo
