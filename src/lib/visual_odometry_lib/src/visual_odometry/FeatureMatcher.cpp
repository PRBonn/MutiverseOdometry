#include "visual_odometry/FeatureMatcher.hpp"
namespace vlo {
FeatureMatcher::FeatureMatcher(Ptr<DescriptorMatcher> matcher) {
  matcher_ = matcher;
}

void FeatureMatcher::match(const Mat &current_descriptors,
                           const Mat &last_frame_descriptors,
                           std::vector<DMatch> &good_matches) {
  std::vector<std::vector<DMatch>> knn_matches_;
  matcher_->clear();
  matcher_->knnMatch(current_descriptors, last_frame_descriptors, knn_matches_,
                     2);

  //-- Filter matches using the Lowe's ratio test
  const float ratio_thresh = 0.8f;  // set as dynamic param later
  for (size_t i = 0; i < knn_matches_.size(); i++) {
    if (knn_matches_[i][0].distance <
        ratio_thresh * knn_matches_[i][1].distance) {
      good_matches.push_back(knn_matches_[i][0]);
    }
  }
}

void FeatureMatcher::match(const VisualFrame::Ptr &current_frame,
                           const VisualFrame::Ptr &reference_frame,
                           std::vector<DMatch> &good_matches) {
  std::vector<std::vector<DMatch>> knn_matches_;
  matcher_->clear();
  matcher_->knnMatch(current_frame->descriptors_, reference_frame->descriptors_,
                     knn_matches_, 2);

  //-- Filter matches using the Lowe's ratio test
  const float ratio_thresh = 0.8f;  // set as dynamic param later
  for (size_t i = 0; i < knn_matches_.size(); i++) {
    if (knn_matches_[i][0].distance <
        ratio_thresh * knn_matches_[i][1].distance) {
      good_matches.push_back(knn_matches_[i][0]);
    } /*else {
      std::cout << " skipped " << std::endl;
       std::cout << current_frame_->key_points_[good_matches[i].queryIdx].pt <<
      std::endl;*/

    //    }
  }
}

void FeatureMatcher::matchToMap(const cv::Mat &current_frame_descriptors,
                                const cv::Mat &map_points_descriptors,
                                std::vector<DMatch> &good_matches,
                                std::vector<int> &potential_new_points) {
  std::vector<std::vector<DMatch>> knn_matches_;
  matcher_->clear();
  matcher_->knnMatch(current_frame_descriptors, map_points_descriptors,
                     knn_matches_, 2);

  //-- Filter matches using the Lowe's ratio test
  const float ratio_thresh = 0.8f;  // set as dynamic param later
  for (size_t i = 0; i < knn_matches_.size(); i++) {
    if (knn_matches_[i][0].distance <
        ratio_thresh * knn_matches_[i][1].distance) {
      good_matches.push_back(knn_matches_[i][0]);
    } else {
      potential_new_points.push_back(knn_matches_[i][0].queryIdx);
    }
  }
}

bool hasBeenSeenBefore(std::vector<DMatch> &good_matches, const DMatch &match) {
  for (auto &good_match : good_matches) {
    if (good_match.trainIdx == match.trainIdx) {
      if (good_match.distance < match.distance) {
        return true;
      } else {
        good_match = match;
        return true;
      }
    }
  }

  return false;
}

void FeatureMatcher::matchToMap(const VisualFrame::Ptr &current_frame,
                                const Map &map_points_in_view,
                                std::vector<DMatch> &good_matches,
                                std::vector<int> &potential_new_points) {
  Points3Dd points3d;
  for (const auto &map_point : map_points_in_view.map_points_) {
    Point3d map_point3d = eigenVectorToPoint3D(current_frame->pose_.inverse() *
                                               map_point->position_);
    points3d.push_back(map_point3d);
  }

  ImagePointsd points2d_map_projected;
  cv::projectPoints(points3d, cv::Mat_<double>::zeros(3, 1),
                    cv::Mat_<double>::zeros(3, 1),
                    current_frame->sensor_->getSensorMatrix(), cv::Mat(),
                    points2d_map_projected);

  cv::Mat check;
  cv::drawKeypoints(current_frame->image_,
                    ImagePointsToKeyPoints(points2d_map_projected), check,
                    cv::Scalar(0, 255, 0));
  cv::drawKeypoints(check, current_frame->getGoodKeyPoints(), check,
                    cv::Scalar(255, 0, 0));
  cv::namedWindow("points from map and good points from current scene",
                  cv::WINDOW_NORMAL);
  cv::imshow("points from map and good points from current scene", check);

  cv::Mat check1, check2;
  cv::drawKeypoints(current_frame->image_,
                    ImagePointsToKeyPoints(points2d_map_projected), check1,
                    cv::Scalar(0, 255, 0));
  cv::drawKeypoints(current_frame->image_, current_frame->getGoodKeyPoints(),
                    check2, cv::Scalar(255, 0, 0));
  cv::imshow("points from map projected", check1);
  cv::imshow("good points from current scene", check2);

  ImagePointsd points2d_current_image =
      keyPointsToImagePoints(current_frame->getGoodKeyPoints());

  cv::Mat points2d_map_projected_mat =
      imagePointsToOpenCVMat(points2d_map_projected);
  cv::Mat points2d_current_mat = imagePointsToOpenCVMat(points2d_current_image);
  cv::Mat current_good_descriptors =
      current_frame->getGoodDescriptors();
  cv::Mat map_good_descriptors = map_points_in_view.getDescriptors();
  std::vector<std::vector<DMatch>> knn_matches_;
  int nearest_neighbours = 2;
  matcher_->clear();
  matcher_->knnMatch(points2d_current_mat, points2d_map_projected_mat,
                     knn_matches_, nearest_neighbours);

  const float ratio_thresh = 0.8f;  // set as dynamic param later
  int count = 0;

  KeyPoints test_new;

  for (size_t i = 0; i < knn_matches_.size(); i++) {
    std::vector<size_t> valid_index;
    for (size_t j = 0; j < nearest_neighbours; j++) {
      if (knn_matches_[i][j].distance < 10.0) {
        valid_index.push_back(j);
      }
    }
    if (valid_index.size() == 0) {
      potential_new_points.push_back(knn_matches_[i][0].queryIdx);
      auto good_index =
          current_frame->good_indexes_[knn_matches_[i][0].queryIdx];
      test_new.push_back(current_frame->getGoodKeyFromIndex(good_index));
    }
    if (valid_index.size() == 1) {
      if (hasBeenSeenBefore(good_matches, knn_matches_[i][valid_index[0]])) {
        continue;
      } else {
        good_matches.push_back(knn_matches_[i][valid_index[0]]);
      }
    }
    if (valid_index.size() == nearest_neighbours) {
      if (hasBeenSeenBefore(good_matches, knn_matches_[i][0])) {
        continue;
      } else {
        cv::Mat descriptor1 =
            map_good_descriptors.row(knn_matches_[i][0].trainIdx);
        cv::Mat descriptor2 =
            map_good_descriptors.row(knn_matches_[i][1].trainIdx);
        cv::Mat descriptor_querry =
            current_good_descriptors.row(knn_matches_[i][0].queryIdx);

        double distance1 = cv::norm(descriptor1 - descriptor_querry, NORM_L2);
        double distance2 = cv::norm(descriptor2 - descriptor_querry, NORM_L2);

        if (distance1 < distance2) {
          if (distance1 < ratio_thresh * distance2) {
            good_matches.push_back(knn_matches_[i][0]);
          }
        } else {
          if (distance2 < ratio_thresh * distance1) {
            good_matches.push_back(knn_matches_[i][1]);
          }
        }
      }
    }
  }

  //  for (const auto &good_match : good_matches) {
  //    std::cout << good_match.trainIdx << " " << good_match.queryIdx
  //              << " distance: " << good_match.distance << std::endl;
  //  }

  cv::Mat img_show;
  cv::drawMatches(current_frame->image_, current_frame->getGoodKeyPoints(),
                  current_frame->image_,
                  ImagePointsToKeyPoints(points2d_map_projected), good_matches,
                  img_show);

  cv::namedWindow("Matchesspace", cv::WINDOW_NORMAL);
  cv::imshow("Matchesspace", img_show);

  drawMatchesImageToMap(current_frame, current_frame->getGoodPoints(),
                        current_frame, points3d, good_matches, "test");

  cv::Mat test;
  cv::drawKeypoints(current_frame->image_, test_new, test,
                    cv::Scalar(0, 0, 255));
  cv::namedWindow("potential new points", cv::WINDOW_NORMAL);
  cv::imshow("potential new points", test);
}

void FeatureMatcher::matchToImage(const Map::Ptr &map, const VisualFrame::Ptr &frame,
                                  std::vector<DMatch> &good_matches) {
  cv::Mat map_descriptors = map->getDescriptors();
  std::vector<std::vector<DMatch>> knn_matches_;
  matcher_->clear();
  matcher_->knnMatch(frame->descriptors_, map_descriptors,
                     knn_matches_, 2);

  //-- Filter matches using the Lowe's ratio test
  const float ratio_thresh = 0.8f;  // set as dynamic param later
  for (size_t i = 0; i < knn_matches_.size(); i++) {
    if (knn_matches_[i][0].distance <
        ratio_thresh * knn_matches_[i][1].distance) {
      good_matches.push_back(knn_matches_[i][0]);
    }
  }
}

}  // namespace vlo
