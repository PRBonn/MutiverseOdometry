#include "lidar_odometry/ImageProjector.hpp"
namespace vlo {
ImageProjector::ImageProjector(const ImageProjectorOptions &options) {
  vertical_scans_ = options.vertical_scans;
  horizontal_scans_ = options.horizontal_scans;
  ang_bottom_ = options.ang_bottom;
  vertical_angle_top_ = options.vertical_angle_top;
  ang_bottom_ = options.ang_bottom;

  const double DEG_TO_RAD = M_PI / 180.0f;
  ang_radian_resolution_X_ = (M_PI * 2) / (horizontal_scans_);
  ang_radian_resolution_Y_ = DEG_TO_RAD * (vertical_angle_top_ - ang_bottom_) /
                             double(vertical_scans_ - 1);
  ang_bottom_ = -(ang_bottom_ - 0.1) * DEG_TO_RAD;
  segment_alpha_X_ = ang_radian_resolution_X_;
  segment_alpha_Y_ = ang_radian_resolution_Y_;
}

void ImageProjector::run(const PointCloud &point_cloud) {
  laser_cloud_in_ = point_cloud;
  findStartEndAngle();
}
void ImageProjector::findStartEndAngle() {
  std::cout << "here " << std::endl;
  open3d::visualization::DrawGeometries(
      {std::make_shared<PointCloud>(laser_cloud_in_)}, "Mesh", 1600, 900);

  auto point = laser_cloud_in_.points_.front();
  auto start_orientation = -std::atan2(point[1], point[0]);

  point = laser_cloud_in_.points_.back();
  auto endOrientation = -std::atan2(point[1], point[0]) + 2 * M_PI;

  if (endOrientation - start_orientation > 3 * M_PI) {
    endOrientation -= 2 * M_PI;
  } else if (endOrientation - start_orientation < M_PI) {
    endOrientation += 2 * M_PI;
  }
  auto orientationDiff = endOrientation - start_orientation;
  std::cout << " difference: " << orientationDiff << std::endl;
}

void ImageProjector::projectPointCloud() {
  // range image projection
  const size_t cloudSize = laser_cloud_in_.points_.size();

  for (size_t i = 0; i < cloudSize; ++i) {
    auto thisPoint = laser_cloud_in_.points_[i];

    double range =
        sqrt(thisPoint[0] * thisPoint[0] + thisPoint[1] * thisPoint[1] +
             thisPoint[2] * thisPoint[2]);

    // find the row and column index in the image for this point
    double verticalAngle = std::asin(thisPoint[2] / range);
    // std::atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y *
    // thisPoint.y));

    int rowIdn = (verticalAngle + ang_bottom_) / ang_radian_resolution_Y_;
    if (rowIdn < 0 || rowIdn >= vertical_scans_) {
      continue;
    }

    double horizonAngle = std::atan2(thisPoint[0], thisPoint[1]);

    int columnIdn = -round((horizonAngle - M_PI_2) / ang_radian_resolution_X_) +
                    horizontal_scans_ * 0.5;

    if (columnIdn >= horizontal_scans_) {
      columnIdn -= horizontal_scans_;
    }

    if (columnIdn < 0 || columnIdn >= horizontal_scans_) {
      continue;
    }

    if (range < 0.1) {
      continue;
    }

    range_mat_(rowIdn, columnIdn) = range;

    // thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

    size_t index = columnIdn + rowIdn * horizontal_scans_;
    full_cloud_.points_.push_back(thisPoint);
    // the corresponding range of a point is saved as "intensity"
    //_full_info_cloud->points[index] = thisPoint;
    //_full_info_cloud->points[index].intensity = range;
  }
}

void ImageProjector::groundRemoval() {
  for (size_t j = 0; j < vertical_scans_; ++j) {
    for (size_t i = 0; i < ground_scan_index_; ++i) {
      size_t lowerInd = j + (i)*vertical_scans_;
      size_t upperInd = j + (i + 1) * vertical_scans_;

      //      if (full_cloud_->points[lowerInd].intensity == -1 ||
      //          full_cloud_->points[upperInd].intensity == -1) {
      //        // no info to check, invalid points
      //        full_cloud_(i, j) = -1;
      //        continue;
      //      }

      float dX =
          full_cloud_.points_[upperInd][0] - full_cloud_.points_[lowerInd][0];
      float dY =
          full_cloud_.points_[upperInd][1] - full_cloud_.points_[lowerInd][1];
      float dZ =
          full_cloud_.points_[upperInd][2] - full_cloud_.points_[lowerInd][2];

      float vertical_angle = std::atan2(dZ, sqrt(dX * dX + dY * dY + dZ * dZ));

      // TODO: review this change

      if ((vertical_angle - sensor_mount_angle_) <= 10 * 180 / M_PI) {
        ground_mat_(i, j) = 1;
        ground_mat_(i + 1, j) = 1;
      }
    }
  }
  // extract ground cloud (_ground_mat == 1)
  // mark entry that doesn't need to label (ground and invalid point) for
  // segmentation note that ground remove is from 0~_N_scan-1, need _range_mat
  // for mark label matrix for the 16th scan
  for (size_t i = 0; i < vertical_scans_; ++i) {
    for (size_t j = 0; j < horizontal_scans_; ++j) {
      if (ground_mat_(i, j) == 1 || label_mat_(i, j) == std::numeric_limits<double>::infinity()) {
        label_mat_(i, j) = -1;
      }
    }
  }

  for (size_t i = 0; i <= ground_scan_index_; ++i) {
    for (size_t j = 0; j < horizontal_scans_; ++j) {
      if (ground_mat_(i, j) == 1){
        auto point_cloud_to_insert = full_cloud_.points_[j + i * horizontal_scans_];
      ground_cloud_.points_.push_back(point_cloud_to_insert);
    }
  }
}
}
}  // namespace vlo
