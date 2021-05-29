#include "ComboVLO/ComboVLO.hpp"

namespace vlo {

ComboVLO::ComboVLO(std::vector<OdometerStruct> odometries,
                   const Transform3D& initial_pose)
    : odometries_(odometries), initial_pose_(initial_pose) {
  trajectory_.push_back(initial_pose);
}

std::vector<vlo::Vec3> colors{Vec3(1.0, 0.0, 0.0),  // red
                              Vec3(1.0, 0.5, 0.0),  // orange
                              Vec3(1.0, 1.0, 0.0),  // yellow
                              Vec3(0.0, 1.0, 0.0),  // green
                              Vec3(0.0, 1.0, 0.0),  // yelllow
                              Vec3(0.0, 0.0, 0.0), Vec3(0.5, 0.5, 0.5)};

void ComboVLO::addVisualAndLidarFrames(IFrame::Ptr& current_frame) {
  if (state_ == VLO_STATE::INIT) {
    current_frame->pose_ = initial_pose_;
    addKeyFrame(current_frame);
    state_ = VLO_STATE::RUNNING;
  } else if (state_ == VLO_STATE::RUNNING) {



    auto [frame_to_frame_transform, trust_result] =
        findFrameToFrameTransformBetween(current_frame, trajectory_nodes_.back());
    for (auto& odometry : odometries_) {
      odometry.odometer->last_transform_ = frame_to_frame_transform.matrix();
    }

    Transform3D current_pose = Transform3D::Identity();
    frame_to_frame_transform_ = frame_to_frame_transform;
    std::cout << "Chosen: " << std::endl;
    std::cout << frame_to_frame_transform.matrix() << std::endl;
    current_pose = trajectory_.back() * frame_to_frame_transform;
    current_frame->pose_ = current_pose;
    std::cout << "current pose: " << current_frame->pose_.matrix() << std::endl;
    trajectory_.push_back(current_pose);
    addKeyFrame(current_frame);
  }
}
std::tuple<Transform3D, bool> ComboVLO::findFrameToFrameTransformBetween(
    IFrame::Ptr& current_frame, IFrame::Ptr& reference_frame) {
  std::vector<std::pair<Transform3D, bool>> frame_to_frame_transforms;
  Eigen::VectorXd errors = Eigen::VectorXd::Zero(odometries_.size());
  for (auto& odometry : odometries_) {
    auto [frame_to_frame_transform, trust_result] =
        odometry.odometer->findFrameToFrameTransformBetween(current_frame,
                                                            reference_frame);
    frame_to_frame_transforms.push_back(std::pair<Transform3D, bool>(
        frame_to_frame_transform.matrix(), trust_result));
  }

  frame_to_frame_transforms.push_back(
      std::pair<Transform3D, bool>(frame_to_frame_transform_, true));

  Transform3D const_velocity_model = Transform3D::Identity();
  const_velocity_model.translation() = frame_to_frame_transform_.translation();
  const_velocity_model.matrix().block<3, 3>(0, 0) = Mat33::Identity();

  frame_to_frame_transforms.push_back(
      std::pair<Transform3D, bool>(const_velocity_model, true));

  return chooseOdometry(frame_to_frame_transforms, current_frame,
                        reference_frame);
}

double Exp(double x)  // the functor we want to apply
{
  return std::exp(x);
}

Eigen::VectorXd softMax(const Eigen::VectorXd& vector) {
  auto exp_vector(vector);
  exp_vector.array().exp();
  return exp_vector / exp_vector.sum();
}

std::pair<Transform3D, bool> ComboVLO::chooseOdometry(
    const std::vector<std::pair<Transform3D, bool>>& frame_to_frame_transforms,
    IFrame::Ptr& current_frame, IFrame::Ptr& reference_frame) {
  Transform3D frame_to_frame_transform = Transform3D::Identity();
  if (choose_odometry_mode_ == ChooseOdometry::BASED_ON_GT) {
    Eigen::VectorXd gt_errors =
        Eigen::VectorXd::Zero(frame_to_frame_transforms.size());
    for (size_t i = 0; i < frame_to_frame_transforms.size(); i++) {
      std::cout << i << " th transform " << std::endl;
      auto diff = gt_transform_.inverse() * frame_to_frame_transforms[i].first;
      std::cout << "Proposal: " << std::endl;
      std::cout << frame_to_frame_transforms[i].first.matrix() << std::endl;
      std::cout << "Difference with gt for the best transformations: "
                << std::endl;
      std::cout << diff.translation().norm() << std::endl;
      gt_errors(i) = diff.translation().norm();
    }
    int index;
    auto smallest_erro = gt_errors.minCoeff(&index);
    index_ = index;
    frame_to_frame_transform = frame_to_frame_transforms[index].first;
    return std::pair<Transform3D, bool>(frame_to_frame_transform, true);
  }
  if (choose_odometry_mode_ == ChooseOdometry::ONLY_ONE) {
    std::cout << "Choose odometry: ONLY_ONE, first transform is chosen"
              << std::endl;
    return frame_to_frame_transforms[0];
  }

  if (choose_odometry_mode_ == ChooseOdometry::VISION_LIDAR_ERROR_METRIC) {
    std::vector<double> error(frame_to_frame_transforms.size(), 0.0);
    std::vector<std::pair<double, double>> results(
        frame_to_frame_transforms.size());
    Eigen::VectorXd lidar_errors =
        Eigen::VectorXd::Zero(frame_to_frame_transforms.size());
    Eigen::VectorXd vision_errors =
        Eigen::VectorXd::Zero(frame_to_frame_transforms.size());

    for (size_t i = 0; i < frame_to_frame_transforms.size(); i++) {
      if (frame_to_frame_transforms[i].second == false) {
        lidar_errors(i) = 100.0;
        vision_errors(i) = 100.0;
      } else {
        auto errors = evaluateErrorOverlapBetween(
            current_frame, reference_frame, frame_to_frame_transforms[i].first);
        lidar_errors(i) = errors.first;
        vision_errors(i) = errors.second;
      }
    }
    std::cout << "Lidar Before softmax: " << lidar_errors.transpose()
              << std::endl;
    lidar_errors = softMax(lidar_errors);  // lidar_errors / lidar_errors.sum();
    std::cout << "Lidar errors softmax: " << lidar_errors.transpose()
              << std::endl;
    std::cout << "vision Before softmax: " << vision_errors.transpose()
              << std::endl;
    vision_errors =
        softMax(vision_errors);  // vision_errors / vision_errors.sum();
    std::cout << "vision_errors softmax" << vision_errors.transpose()
              << std::endl;
    std::cout << "After revaluation:" << std::endl;
    Eigen::VectorXd errors = lidar_errors + vision_errors;
    std::cout << "percentage conversion:" << errors.transpose() << std::endl;
    int index;

    auto smallest_erro = errors.minCoeff(&index);
    index_ = index;
    std::cout << "smallest error: " << smallest_erro << std::endl;

    frame_to_frame_transform = frame_to_frame_transforms[index].first;

    std::ofstream myfile;
    myfile.open("errors_lidar.txt", std::ios_base::app);
    myfile << errors.transpose() << "\n";
    myfile.close();
    return std::pair<Transform3D, bool>(frame_to_frame_transform, true);
  }

  if (choose_odometry_mode_ == ChooseOdometry::SANITY_CHECKER_METRIC) {

    auto t_start = std::chrono::high_resolution_clock::now();

    auto map = getLocalMap(reference_frame);

//    open3d::io::WritePointCloudToPCD("map.pcd", map,
//                                     {true});
//    map.PaintUniformColor(Vec3(0,1,0));
//    auto adding_small_map = std::make_shared<PointCloud>(current_frame->point_cloud_);
//    adding_small_map->Transform(frame_to_frame_transforms[0].first.matrix());
//    adding_small_map->PaintUniformColor(Vec3(1,0,1));

//    open3d::io::WritePointCloudToPCD("map_with_proposal.pcd", map+*adding_small_map,
//                                     {true});


    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
    std::cout << "Get local map in: " << elapsed_time_ms << std::endl;
    std::cout << "Sanity Checks!" << std::endl;
    std::cout << "Prev velocity: " << sanity_checker_.previous_velocity_
              << std::endl;
    std::cout << "Max acceleration: " << sanity_checker_.max_acceleration_
              << std::endl;
    Eigen::VectorXd vector_acceleration =
        Eigen::VectorXd::Zero(frame_to_frame_transforms.size());
    Eigen::VectorXd vector_velocity =
        Eigen::VectorXd::Zero(frame_to_frame_transforms.size());
    Eigen::VectorXd vside_velocity =
        Eigen::VectorXd::Zero(frame_to_frame_transforms.size());

    std::vector<vlo::Vec3> velocity_vector_vector;

    Eigen::VectorXd lidar_errors =
        Eigen::VectorXd::Zero(frame_to_frame_transforms.size());

    for (size_t i = 0; i < frame_to_frame_transforms.size(); i++) {
      std::cout << "Proposal: " << std::endl;
      std::cout << frame_to_frame_transforms[i].first.matrix() << std::endl;
      auto [acceleration, velocity, isMaxAccelerationCrossed, velocity_vector] =
          sanity_checker_.dynamicsChecker(frame_to_frame_transforms[i].first,
                                          current_frame, reference_frame);
      velocity_vector_vector.push_back(velocity_vector);
      //       auto [Vside, isMaxSideVelocityCrossed] =
      //       sanity_checker_.checkAckermann(frame_to_frame_transforms[i].first,
      //          current_frame, reference_frame);

      vector_acceleration(i) = std::abs(acceleration);
      vector_velocity(i) = velocity;
      //          vside_velocity(i) = Vside;

      if (!isMaxAccelerationCrossed /* and !isMaxSideVelocityCrossed*/) {
        std::cout << "NOT CROSSED" << std::endl;
        auto errors = evaluateErrorOverlapBetween(
            current_frame, map, frame_to_frame_transforms[i].first);
        lidar_errors(i) = errors.first;
        //        ackermann_errors(i) =
        //        sanity_checker_.checkAckermann(frame_to_frame_transforms[i].first,
        //                                                              current_frame,
        //                                                              reference_frame);

      } else {
        std::cout << "MAX ACCELERATION CROSSED" << std::endl;
        lidar_errors(i) = 100.0;
      }
    }
    lidar_errors = softMax(lidar_errors);
    std::cout << "lidar errors: " << lidar_errors.transpose() << std::endl;
    Eigen::VectorXd errors = lidar_errors;
    std::cout << "errors: " << errors.transpose() << std::endl;
    int index;
    auto smallest_erro = errors.minCoeff(&index);
    std::cout << "smallest error: " << std::endl;
    std::cout << smallest_erro << std::endl;
    std::cout << "INDEX: " << index << std::endl;
    index_ = index;
    velocity = vector_velocity(index_);
    acceleration_ = vector_acceleration(index_);
    ackermann_velocity_ = vside_velocity(index_);

    frame_to_frame_transform = frame_to_frame_transforms[index].first;
    sanity_checker_.previous_velocity_ = vector_velocity(index);
    sanity_checker_.previous_velocity_vector_ = velocity_vector_vector[index];
    acceleration_vector_ = velocity_vector_vector[index];

    return std::pair<Transform3D, bool>(frame_to_frame_transform, true);
  }

  std::cout << "It shouldn't happen, since no method for transform discovery "
               "has been detected..."
            << std::endl;
  exit(EXIT_FAILURE);
}

PointCloud ComboVLO::getLocalMap(const IFrame::Ptr& reference_frame) {
  PointCloud map;
  int i = 0;
  for (const auto node : trajectory_nodes_) {

    auto transform =  node->pose_.inverse() * trajectory_nodes_.back()->pose_ ;
//    std::cout << "Map transform\n" << transform.matrix() << std::endl;
//    std::cout << "nose pose" << node->pose_.inverse().matrix() << std::endl;
//    std::cout << "last tf  " << trajectory_nodes_.back()->pose_.matrix() << std::endl;
//    node->point_cloud_.PaintUniformColor(colors[i % colors.size()]);
    auto submap = std::make_shared<PointCloud>(node->point_cloud_);
    submap->Transform(transform.inverse().matrix());
    map = map + *submap;
      i++;
  }
//  std::cout << "Saving map..." << std::endl;
  map.VoxelDownSample(0.25);


   return map;
}

void ComboVLO::addKeyFrame(const IFrame::Ptr& frame) {
  pushFrameToHistory(frame);
}

float distancePointFromLine(const Vec3& point, const Vec3& line) {
  // Line is given as a*x + b*y + c = 0
  return std::fabs(line(0) * point(0) + line(1) * point(1) + line(2)) /
         std::sqrt(line(0) * line(0) + line(1) * line(1));
}

std::pair<double, double> ComboVLO::evaluateErrorOverlapBetween(
    const IFrame::Ptr current, const IFrame::Ptr& reference,
    const Transform3D& transform_from_current_reference) {
  auto point_cloud_transformed =
      std::make_shared<PointCloud>(reference->point_cloud_);
  point_cloud_transformed->Transform(
      transform_from_current_reference.inverse().matrix());
  auto results = open3d::pipelines::registration::EvaluateRegistration(
      *point_cloud_transformed, current->point_cloud_, 0.5);

  //  std::cout << "ComboVLO: Correspondence set: " <<
  //  results.correspondence_set_.size() << std::endl; auto translation =
  //  transform_from_current_reference.translation(); Eigen::Matrix3d t_matrix;
  //  t_matrix << 0.0, -translation(2), translation(1), translation(2), 0.0,
  //      -translation(0), -translation(1), translation(0), 0.0;

  //  Eigen::Matrix3d essential_matrix =
  //      transform_from_current_reference.rotation().matrix() * t_matrix;

  //  std::vector<cv::Point2f> points_reference;
  //  std::vector<cv::Point2f> points_current;
  //    KeyPoints test_ref;
  //    KeyPoints test_cur;
  //    std::cout << "COMBOVLO number of keypoints: "
  //              << reference->getGoodKeyPoints().size() << std::endl;
  //    std::cout << "cur keypoints: " << current->getGoodKeyPoints().size()
  //              << std::endl;

  //  for (int i = 0; i < reference->getGoodKeyPoints().size(); i++) {
  //    points_reference.push_back(reference->getGoodKeyPoints()[i].pt);
  //    points_current.push_back(current->getGoodKeyPoints()[i].pt);
  //  }

  //  cv::Mat out_img_ref;
  //  cv::drawKeypoints(reference->image_, test_ref, out_img_ref);
  //  cv::imshow("out ref img", out_img_ref);

  //  cv::Mat out_img_cur;
  //  cv::drawKeypoints(current->image_, test_cur, out_img_cur);
  //  cv::imshow("out cur img", out_img_cur);

  //  double sum = 0.0;
  //  double count = 0.0;
  //
  //  std::cout << "Essential matrix: " << essential_matrix << std::endl;
  //  std::cout << "rotation: " << transform_from_current_reference.rotation()
  //            << std::endl;
  //  std::cout << "translation: "
  //            << transform_from_current_reference.translation();
  //  std::cout << "translation matrix: " << t_matrix << std::endl;

  //  for (int i = 0; i < points_current.size(); i++) {
  //    Eigen::Vector3d reference_point, current_point;
  //    reference_point(0) = points_reference[i].x;
  //    reference_point(1) = points_reference[i].y;
  //    reference_point(2) = 1.0;

  //    current_point(0) = points_current[i].x;
  //    current_point(1) = points_current[i].y;
  //    current_point(2) = 1.0;

  //    auto line = essential_matrix * reference_point ;

  //    if (distancePointFromLine(current_point, line.normalized()) <
  //        0.5) {  // 2.0
  //      sum = sum + distancePointFromLine(current_point, line.normalized());
  //      count = count + 1.0;
  //    }
  //  }

  //  std::cout << "how many points taken into account: " <<
  //  points_current.size()
  //            << std::endl;
  //  auto vision_rmse =
  //      std::sqrt(sum / static_cast<double>(points_current.size()));
  //  if (std::isnan(vision_rmse) or points_current.size() <10) {
  //    std::cout << vision_rmse << std::endl;
  //    vision_rmse = 100.0;

  //  }

  return std::pair<double, double>(results.inlier_rmse_, 1
                                   /*vision_rmse*/);  // results.inlier_rmse_;
}

std::pair<double, double> ComboVLO::evaluateErrorOverlapBetween(
    const IFrame::Ptr current, const PointCloud& reference_point_cloud,
    const Transform3D& transform_from_current_reference) {
  std::cout << "Evaluate to the map!" << std::endl;
  auto point_cloud_transformed =
      std::make_shared<PointCloud>(reference_point_cloud);
  point_cloud_transformed->Transform(
      transform_from_current_reference.inverse().matrix());
  auto results = open3d::pipelines::registration::EvaluateRegistration(
      *point_cloud_transformed, current->point_cloud_, 0.5);


  return std::pair<double, double>(results.inlier_rmse_, 1
                                   /*vision_rmse*/);  // results.inlier_rmse_;
}

void ComboVLO::pushFrameToHistory(const IFrame::Ptr& camera_frame) {
  // map_->insertKeyFrame(frame);
  trajectory_nodes_.push_back(camera_frame);
  if (trajectory_nodes_.size() > 10) {
    trajectory_nodes_.erase(trajectory_nodes_.begin());
  }
}

double ComboVLO::calculateCrossCheckingError(
    const IFrame::Ptr& current_frame, const IFrame::Ptr& reference_frame) {}
}  // namespace vlo
