#include "VO_PnP.hpp"

namespace vlo {

VO_PnP::VO_PnP(const cv::Ptr<Feature2D> &descriptor,
               const cv::Ptr<DescriptorMatcher> &matcher,
               const Transform3D &initial_pose = Transform3D::Identity())
    : IVisualOdometer(initial_pose),
      feature_detector_(FeatureDetector(descriptor)),
      feature_matcher_(FeatureMatcher(matcher)),
      map_(std::make_shared<Map>()) {}

/*******************************************************************************
 *
 *                      MAIN METHOD TO PROCESS IMAGE
 *
 * ****************************************************************************/
void VO_PnP::processFrame(VisualFrame::Ptr frame) {
  std::string time_stamp = std::to_string(frame->time_stamp_);
  current_frame_ = frame;
  feature_detector_.calculateKeyPointsAndDescriptors(current_frame_);

  if (vo_state_ == VO_STATE::STARTING) {
    current_frame_->pose_ = initial_pose_;
    addKeyFrame(current_frame_);
    vo_state_ = VO_STATE::INITIALIZING;
  } else if (vo_state_ == VO_STATE::INITIALIZING) {
    findCurrentImageToReferenceImageMatches();
    drawGoodAndBadKeypoints(
        current_frame_, "Bad and Good Keypoints after Matching, current frame",
        0);
    drawGoodAndBadKeypoints(
        reference_frame_,
        "Bad and Good Keypoints after Matching, refernece frame", 0);
    Points3Dd triangulated_points_out;
    imageToimageTransform5point(current_frame_->getGoodPoints(),
                                reference_frame_->getGoodPoints(),
                                triangulated_points_out);
    if (triangulated_points_out.size() > 10) {
      cv::Mat pnp_inliers_mask;
      pnp_run(triangulated_points_out, reference_frame_->getGoodPoints(),
              pnp_inliers_mask);
      Points3Dd very_good_triangulated_points;
      Indexes very_good_indexes_after_triangulation_last_cene,
          very_good_indexes_after_triangulation_current_scene;
      for (int i = 0; i < pnp_inliers_mask.rows; i++) {
        auto index = pnp_inliers_mask.at<int>(i, 0);
        very_good_triangulated_points.push_back(triangulated_points_out[index]);
        very_good_indexes_after_triangulation_last_cene.push_back(
            reference_frame_->good_indexes_[index]);
        very_good_indexes_after_triangulation_current_scene.push_back(
            current_frame_->good_indexes_[index]);
      }
      reference_frame_->updateGoodIndexes(
          very_good_indexes_after_triangulation_last_cene);
      current_frame_->updateGoodIndexes(
          very_good_indexes_after_triangulation_current_scene);
      drawGoodAndBadKeypoints(
          current_frame_,
          "Bad and Good Keypoints after Matching, current frame, after pnp", 0);
      drawGoodAndBadKeypoints(
          reference_frame_,
          "Bad and Good Keypoints after Matching, refernece frame, after pnp",
          0);
      saveValuableMapPoints(very_good_triangulated_points,
                            current_frame_->getGoodPoints());
    } else {
      constantVelocityModel();
    }
//    if (getMap().size() > 1000) {
//      vo_state_ = VO_STATE::RUNNING;
//    }
    addKeyFrame(current_frame_);
  } else if (vo_state_ == VO_STATE::RUNNING) {
      //TODO: for the time being mapping part doesn't work.
    std::cout << " R!U!N!N!I!N!G " << std::endl;

    findCurrentImageToReferenceImageMatches();
    //    Points3Dd good_triangulated_points;
    //    imageToimageTransform5point(current_frame_->getGoodPoints(),
    //                                reference_frame_->getGoodPoints(),
    //                                good_triangulated_points);
    cv::Mat pnp_inliers_mask;
    std::vector<DMatch> good_matches_image;

    Map::Ptr local_map_in_world_frame = std::make_shared<Map>();
    findMapPointsInCameraViewBasedOnGeometry(reference_frame_,
                                             *local_map_in_world_frame);
    Points3Dd points3d;
    drawMap(reference_frame_, reference_frame_->camera_->getCameraMatrix(),
            local_map_in_world_frame->getMap(),
            "Local map with visibility times");
    local_map_in_world_frame->getPointsExpressedInGivenFrame(reference_frame_,
                                                             points3d);
    drawProjectedPoints(points3d, reference_frame_,
                        "Local Map in running projeceted on reference frame");
    drawGoodKeyPoints(reference_frame_, "Good keypoints in reference frame", 0,
                      cv::Scalar(0, 0, 255));
    std::cout << " update  " << std::endl;
    cv::waitKey(0);
    feature_matcher_.matchToImage(local_map_in_world_frame, current_frame_,
                                  good_matches_image);

    //    drawMatchesImageToMap(reference_frame_,
    //    reference_frame_->getGoodPoints(),
    //                          reference_frame_, points3d, good_matches_image,
    //                          "Compare");
    ImagePointsd goog_points;
    Points3Dd map_points3d;
    for (size_t i = 0; i < good_matches_image.size(); i++) {
      goog_points.push_back(current_frame_->key_points_[good_matches_image[i].queryIdx].pt);
      map_points3d.push_back(points3d[good_matches_image[i].trainIdx]);
    }


   pnp_run(map_points3d, goog_points, pnp_inliers_mask);
 std::cout << "pnp inlier mask " << pnp_inliers_mask.size() << std::endl;
    //    saveValuableMapPoints(good_triangulated_points,
    //                          current_frame_->getGoodPoints());
      addKeyFrame(current_frame_);
  }
}

double VO_PnP::calculateCrossCheckingError(const IFrame::Ptr & current_frame, const IFrame::Ptr & reference_frame)
{

}


void VO_PnP::saveValuableMapPoints(const Points3Dd &good_triangulated_points,
                                   const ImagePointsd &good_image_points) {
  Map map_points_in_view;
  Indexes potential_new_points_indexes;
  findMapPointsInCameraViewBasedOnGeometry(current_frame_, map_points_in_view);

  if (map_points_in_view.size() == 0) {
    pushCurrentTriangulatedPointsToMap(good_triangulated_points,
                                       good_image_points);
  } else {
    Points3Dd points3d;
    map_points_in_view.getPointsExpressedInGivenFrame(current_frame_, points3d);
    std::vector<DMatch> current_matches_frame_to_map;
    feature_matcher_.matchToMap(current_frame_, map_points_in_view,
                                current_matches_frame_to_map,
                                potential_new_points_indexes);

    KeyPoints image_points_current_matched_to_map;
    for (const auto &current_match_frame_to_map :
         current_matches_frame_to_map) {
      map_points_in_view[current_match_frame_to_map.trainIdx]->visible_times_++;

      ImagePointsd goog_points;
      auto index =
          current_frame_->good_indexes_[current_match_frame_to_map.queryIdx];
      cv::KeyPoint point = current_frame_->getGoodKeyFromIndex(index);
      image_points_current_matched_to_map.push_back(point);
    }

    Points3Dd new_3dpoints_to_map;
    ImagePointsd new_2dpoints_to_map;

    for (const auto &potential_new_point_index : potential_new_points_indexes) {
      new_3dpoints_to_map.push_back(
          good_triangulated_points[potential_new_point_index]);
      new_2dpoints_to_map.push_back(
          good_image_points[potential_new_point_index]);
    }
    pushCurrentTriangulatedPointsToMap(new_3dpoints_to_map,
                                       new_2dpoints_to_map);
    findMapPointsInCameraViewBasedOnGeometry(current_frame_,
                                             map_points_in_view);
    drawMap(current_frame_, current_frame_->camera_->getCameraMatrix(),
            map_points_in_view.getMap());
    drawProjectedPoints(new_3dpoints_to_map, current_frame_, "hmm these point");
    std::cout << "Size map points: " << map_points_in_view.size() << std::endl;
  }
}

void VO_PnP::constantVelocityModel() {
  std::cout << "Constant Velocity model!" << std::endl;

  auto last_transform = getFrameToFrameTransformation();

  last_transform.matrix().block<3, 1>(0, 3) =
      scale_ * last_transform.translation().normalized();
  last_transform.matrix().block<3, 3>(0, 0) = Mat33::Identity();

  Transform3D current_pose = reference_frame_->pose_ * last_transform;
  Eigen::Quaterniond q_current(current_pose.rotation().matrix());
  q_current.normalize();
  current_pose.matrix().block<3, 3>(0, 0) = q_current.toRotationMatrix();
  current_frame_->pose_ = current_pose;
}

void VO_PnP::pnp_run(const Points3Dd &points_3D,
                     const ImagePointsd &points_image, cv::Mat &mask_out) {
  cv::Mat R_vec, t, pnp_inliers_mask, rotation;
  Transform3D current_transformation = Transform3D::Identity();
  Mat33 d_rotation_eigen;
  Vec3 d_translation_eigen;

  cv::solvePnPRansac(points_3D, points_image,
                     current_frame_->camera_->getCameraMatrix(), cv::Mat(),
                     R_vec, t, false, 50, 0.05, 0.999, mask_out);
  cv::Rodrigues(R_vec, rotation);
  cv2eigen(rotation, d_rotation_eigen);
  cv2eigen(t, d_translation_eigen);
  d_translation_eigen.normalize();
  Eigen::Quaterniond q(d_rotation_eigen);
  q.normalize();
  d_rotation_eigen = q.toRotationMatrix();

  std::cout << "d translation " << d_translation_eigen << std::endl;
  std::cout << "r dotation " << d_rotation_eigen << std::endl;
  std::cout << "d rotation euler "
            << d_rotation_eigen.eulerAngles(0, 1, 2) * 180 / M_PI << std::endl;

  current_transformation.matrix().block<3, 1>(0, 3) =
      scale_ * d_translation_eigen;
  current_transformation.matrix().block<3, 3>(0, 0) = d_rotation_eigen;
  Transform3D current_pose = reference_frame_->pose_ * current_transformation;
  Eigen::Quaterniond q_current(current_pose.rotation().matrix());
  q_current.normalize();
  current_pose.matrix().block<3, 3>(0, 0) = q_current.toRotationMatrix();
  current_frame_->pose_ = current_pose;
}

void VO_PnP::imageToimageTransform5point(
    const ImagePointsd &good_points_current_scene,
    const ImagePointsd &good_points_last_scene,
    Points3Dd &triangulated_points_out) {
  cv::Mat mask, triangulated_points;

  auto essential_matrix =
      cv::findEssentialMat(good_points_current_scene, good_points_last_scene,
                           current_frame_->camera_->getCameraMatrix(),
                           cv::RANSAC, 0.999, 0.05, mask);
  Mat t;
  Mat rotation = Mat_<double>::eye(3, 3);
  cv::recoverPose(essential_matrix, good_points_current_scene,
                  good_points_last_scene,
                  current_frame_->camera_->getCameraMatrix(), rotation, t, 50.0,
                  mask, triangulated_points);

  Indexes good_indexes_after_triangulation_last_cene,
      good_indexes_after_triangulation_current_scene;
  for (int i = 0; i < triangulated_points.cols; i++) {
    if (!static_cast<bool>(mask.at<uchar>(i))) {
      continue;
    }
    cv::Mat X = triangulated_points.col(i);
    X /= X.at<double>(3, 0);
    cv::Point3d pt3d_in_world(X.at<double>(0, 0), X.at<double>(1, 0),
                              X.at<double>(2, 0));
    triangulated_points_out.push_back(pt3d_in_world);
    good_indexes_after_triangulation_last_cene.push_back(
        reference_frame_->good_indexes_[i]);
    good_indexes_after_triangulation_current_scene.push_back(
        current_frame_->good_indexes_[i]);
  }
  current_frame_->updateGoodIndexes(
      good_indexes_after_triangulation_current_scene);
  reference_frame_->updateGoodIndexes(
      good_indexes_after_triangulation_last_cene);
  current_frame_->points3D_ = triangulated_points_out;
  Mat33 d_rotation_eigen;
  Vec3 d_translation_eigen;
  cv2eigen(rotation, d_rotation_eigen);
  cv2eigen(t, d_translation_eigen);

  Eigen::Quaterniond q(d_rotation_eigen);
  q.normalize();
  d_rotation_eigen = q.toRotationMatrix();
  Transform3D current_transformation = Transform3D::Identity();
  current_transformation.matrix().block<3, 1>(0, 3) = d_translation_eigen;
  current_transformation.matrix().block<3, 3>(0, 0) = d_rotation_eigen;
  Points3Dd triangulated_points_references;
  for (int i = 0; i < triangulated_points.cols; i++) {
    if (!static_cast<bool>(mask.at<uchar>(i))) {
      continue;
    }
    cv::Mat X = triangulated_points.col(i);
    X /= X.at<double>(3, 0);
    cv::Point3d pt3d_in_world(X.at<double>(0, 0), X.at<double>(1, 0),
                              X.at<double>(2, 0));
    Vec3 position =
        current_transformation *
        Vec3(pt3d_in_world.x, pt3d_in_world.y, pt3d_in_world.z).homogeneous();
    triangulated_points_references.push_back(eigenVectorToPoint3D(position));
  }

  drawProjectedPoints(current_frame_->points3D_, current_frame_,
                      "After 5 point algorithm on current frame");
  drawProjectedPoints(triangulated_points_references, reference_frame_,
                      "After 5 point algorithm on reference frame");
  if (std::abs(cv::determinant(rotation)) - 1.0 > 1e-07) {
    std::cerr << "det(R) != +-1.0, this is not a rotation matrix" << std::endl;
    return;
  }
}

boost::circular_buffer<MapPoint::Ptr> VO_PnP::getMap() {
  return map_->getMap();
}
void VO_PnP::pushCurrentTriangulatedPointsToMap(
    const Points3Dd &triangulated_points,
    const ImagePointsd &good_points_current_scene) {
  auto transform_to_world_frame = current_frame_->pose_;
  for (int i = 0; i < triangulated_points.size(); i++) {
    cv::Vec3b color =
        current_frame_->image_.at<Vec3b>(good_points_current_scene[i]);
    Vec3 position =
        transform_to_world_frame * Vec3(scale_ * triangulated_points[i].x,
                                        scale_ * triangulated_points[i].y,
                                        scale_ * triangulated_points[i].z)
                                       .homogeneous();
    Vec3 viewing_direction =
        Vec3::Identity();  // TODO: maybe somehow could be used
    auto descriptor = current_frame_->getGoodDescriptorFromIndex(i);
    MapPoint::Ptr point3D = std::make_shared<MapPoint>(
        position, descriptor, viewing_direction, color);
    map_->addMapPoint(point3D);
  }
}

void VO_PnP::findMapPointsInCameraViewBasedOnGeometry(const VisualFrame::Ptr &frame,
                                                      Map &map_points_in_view) {
  const auto map = getMap();
  if (map.size() == 0) {
    std::cout << "No points in the map! " << std::endl;
    return;
  }
  Points3Dd map_points_on_image;
  KeyPoints reprojected_keypoints;
  for (const auto &map_point : map) {
    auto local_map_point_position_eigen =
        frame->pose_.inverse() * map_point->position_;
    cv::Point3d local_map_point_position =
        eigenVectorToPoint3D(local_map_point_position_eigen);
    if (local_map_point_position.z > 0.0) {
      cv::Mat point2d_mat;
      cv::Point2d point2d = projectPointToImagePlane(
          local_map_point_position, frame->camera_->getCameraMatrix());
      if (isInImage(frame, point2d)) {
        map_points_in_view.addMapPoint(map_point);
      }
    }
  }
}

void VO_PnP::setupInitPose(Transform3D initial_pose) {
  initial_pose_ = initial_pose;
}

void VO_PnP::calculateScaleFromGroundTruth(Vec3 d_translation) {
  auto length = d_translation.norm();
  scale_ = 1;  // length;
}

void VO_PnP::addKeyFrame(VisualFrame::Ptr frame) {
  map_->insertKeyFrame(frame);
  pushFrameToHistory(frame);
  reference_frame_ = frame;
}

void VO_PnP::findCurrentImageToReferenceImageMatches() {
  IndexMatches good_matches;
  feature_matcher_.match(current_frame_->descriptors_,
                         reference_frame_->descriptors_, good_matches);
  Indexes good_current_indexes, good_last_indexes;
  for (size_t i = 0; i < good_matches.size(); i++) {
    good_current_indexes.push_back(good_matches[i].queryIdx);
    good_last_indexes.push_back(good_matches[i].trainIdx);
  }
  current_frame_->updateGoodIndexes(good_current_indexes);
  reference_frame_->updateGoodIndexes(good_last_indexes);
}

};  // namespace vlo
