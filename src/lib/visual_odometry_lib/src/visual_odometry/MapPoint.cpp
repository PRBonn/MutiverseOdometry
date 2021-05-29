#include "visual_odometry/MapPoint.hpp"

namespace vlo {

int MapPoint::factory_id_ = 0;

MapPoint::MapPoint(const Vec3 &position, const cv::Mat &descriptor,
                   const Vec3 &viewing_direction, const cv::Vec3b &color)
    : position_(position),
      descriptor_(descriptor),
      viewing_direction_(viewing_direction),
      color_(color),
      visible_times_(1)  //  good_(true), , matched_times_(1)
{
  id_ = factory_id_++;
}

void MapPoint::setPosition(const vlo::Vec3 &positon) { position_ = positon; }

vlo::Vec3 MapPoint::getPosition() { return position_; }
cv::Vec3b MapPoint::getColor() { return color_; }

};  // namespace vlo
