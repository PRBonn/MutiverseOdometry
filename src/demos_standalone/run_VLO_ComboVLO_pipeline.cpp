
#include <algorithm>

#include "ComboVLO/ComboVLO.hpp"
#include "GicpLO/GicpLO.hpp"
#include "HuangsVLO/HuangVLO.hpp"
#include "HuangsVLO/LidarOdometer1DOF.hpp"
#include "LidarOdometerFrameToFrame.hpp"
#include "NdtLO/NdtLO.hpp"

//#include "RvizBridgeViewer.hpp"
#include "ColorBasedVLO/ColorBasedVLO.hpp"
#include "VLO6dof/VLO6dof.hpp"
#include "VO_FrameToFrame_5point.hpp"
#include "VisualLidarFrame.hpp"
#include "open3d/Open3D.h"
#include "reading_utils/config_reader.hpp"
////////////////////////////////

using namespace std;

vlo::ComboVLO::ChooseOdometry getModeFromString(const std::string& mode) {
  std::string mode_lower =
      mode;  // TODO think whether it could be done in a more neat way
  std::transform(mode_lower.begin(), mode_lower.end(), mode_lower.begin(),
                 ::tolower);
  if (mode_lower == "first_not_failed") {
    std::cout << "Choosing first_not_failed metric for odometry choosing"
              << std::endl;

    return vlo::ComboVLO::ChooseOdometry::FIRST_NOT_FAILED;
  }
  if (mode_lower == "vision_lidar_error_metric") {
    std::cout
        << "Choosing vision_lidar_error_metric metric for odometry choosing"
        << std::endl;

    return vlo::ComboVLO::ChooseOdometry::VISION_LIDAR_ERROR_METRIC;
  }
  if (mode_lower == "based_on_git") {
    std::cout << "Choosing based_on_git metric for odometry choosing"
              << std::endl;

    return vlo::ComboVLO::ChooseOdometry::BASED_ON_GT;
  }
  if (mode_lower == "sanity_checker_metric") {
    std::cout << "Choosing sanity_checker_metricfor odometry choosing"
              << std::endl;

    return vlo::ComboVLO::ChooseOdometry::SANITY_CHECKER_METRIC;
  }
  if (mode_lower == "only_one") {
    std::cout << "Choosing only_onemetric for odometry choosing" << std::endl;
    return vlo::ComboVLO::ChooseOdometry::ONLY_ONE;
  }

  std::cerr << "\033[1;31mWrong mode for choosing odometry \033[0m"
            << std::endl;
  exit(EXIT_FAILURE);
}

int main(int argc, char* argv[]) {
  //  if (argc != 2) {
  //    std::cerr
  //        << "\033[1;31mThe running should be:\n./main /path/to/yaml/file
  //        \033[0m"
  //        << std::endl;
  //    return 0;
  //  }
  std::string config_file_path =
      "/home/andrzej/code_workspace/combo_vlo/src/demos_standalone/config/"
      "config.yaml";  // argv[1];
  std::cout << "-------------------------------Starting reading config file "
               "-----------------------"
            << std::endl;
  ConfigReader config_reader = readConfigFromYamlFile(config_file_path);
  std::cout << "---------------------------------------------------------------"
               "---------------------"
            << std::endl;

  //  ros::init(argc, argv, "combo_pipeline");
  //  ros::NodeHandle nh;
  //  ros::Rate loop_rate(1);

  std::vector<vlo::OdometerStruct> vlo_odometries;
  bool color_pc_include = false;
  /*******************************
   GROUNDTRUTH
 *******************************/

  auto ground_truth_poses = read_poses(config_reader.ground_truth_data_file);
  //          for (auto &ground_truth_pose : ground_truth_poses) {
  //            ground_truth_pose.prerotate(vlo::ROTY_90).prerotate(vlo::ROTX_MINUS_90);
  //          }

  /*******************************
   SETING UP VISUALIZER BRIDGE
 *******************************/

  //  std::shared_ptr<RvizBridgeViewer> rviz_bridge_viewer;
  //  rviz_bridge_viewer = std::make_shared<RvizBridgeViewer>();
  //  std::unique_ptr<std::thread> ptr_rviz_bridge_viewer;
  //  ptr_rviz_bridge_viewer =
  //      std::make_unique<std::thread>(&RvizBridgeViewer::run,
  //      rviz_bridge_viewer);

  /*******************************
   VISUAL ODOMETRY CONFIGURATION
 *******************************/
  cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3) << config_reader.fx, 0, config_reader.cx, 0,
       config_reader.fy, config_reader.cy, 0, 0, 1);
  vlo::Camera::Ptr camera = std::make_shared<vlo::Camera>(camera_matrix);

  vlo::Transform3D camera_lidar_transform = vlo::Transform3D::Identity();
  camera_lidar_transform.matrix() << config_reader.r1, config_reader.r2,
      config_reader.r3, config_reader.t1, config_reader.r4, config_reader.r5,
      config_reader.r6, config_reader.t2, config_reader.r7, config_reader.r8,
      config_reader.r9, config_reader.t3, 0, 0, 0, 1;
  //  assert(camera_lidar_transform.rotation().determinant() != 1.0);

  for (auto it = config_reader.odometries_parameteres_.begin();
       it < config_reader.odometries_parameteres_.end(); ++it) {
    if (it->type == "huang_vlo") {
      std::cout << "Huang VLO activated" << std::endl;
      Ptr<Feature2D> descriptor_ptr;
      descriptor_ptr = xfeatures2d::SIFT::create(
          it->sift_nfeatures, it->sift_nOctaveLayers,
          it->sift_contrastThreshold, it->sift_edgeThreshold, it->sift_sigma);

      vlo::FeatureDetector feature_detector(descriptor_ptr);
      Ptr<DescriptorMatcher> matcher =
          DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
      vlo::VO_FrameToFrame_5point::Ptr visual_odometer_5dof =
          std::make_shared<vlo::VO_FrameToFrame_5point>(
              matcher, feature_detector,
              ground_truth_poses[config_reader.from_image]);
      visual_odometer_5dof->debug_ = config_reader.debug;
      visual_odometer_5dof->verbose_ = config_reader.verbose;
      vlo_odometries.push_back(
          vlo::getPointerAndOrientationToHuangsMethodAndInitialize(
              visual_odometer_5dof,
              ground_truth_poses[config_reader.from_image], config_reader.debug,
              config_reader.verbose));
    }

    if (it->type == "point2plane_lo") {
      std::cout << "Point to Plane activated. " << std::endl;
      vlo_odometries.push_back(vlo::getLOFrameToFrameAndInitialize(
          ground_truth_poses[config_reader.from_image],
          "/config_point2plane.yaml", config_reader.debug,
          config_reader.verbose));
    }

    if (it->type == "colorbased_vlo") {
      std::cout << "Color Based VLO activated. " << std::endl;
      vlo_odometries.push_back(vlo::getColorBasedVLOFramAndInitialize(
          ground_truth_poses[config_reader.from_image], config_reader.debug,
          config_reader.verbose));
      color_pc_include = true;
    }

    if (it->type == "ndt_lo") {
      std::cout << "NDT activated. " << std::endl;
      vlo_odometries.push_back(vlo::getNdtLOAndInitialize(
          ground_truth_poses[config_reader.from_image],
          "/config_point2plane.yaml", config_reader.debug,
          config_reader.verbose));
    }
    if (it->type == "gicp_lo") {
      std::cout << "GICP activated. " << std::endl;
      vlo_odometries.push_back(vlo::getGicpLOAndInitialize(
          ground_truth_poses[config_reader.from_image],
          "/config_point2plane.yaml", config_reader.debug,
          config_reader.verbose));
    }
  }

  /*******************************
  COMBO ODOMETRY CONFIGURATION
  *******************************/
  vlo::ComboVLO::Ptr combo_vlo = std::make_shared<vlo::ComboVLO>(
      vlo_odometries, ground_truth_poses[config_reader.from_image]);
  // combo_vlo->camera_lidar_transform_ = camera_lidar_transform;
  combo_vlo->choose_odometry_mode_ =
      getModeFromString(config_reader.sanity_checker_mode);
  std::cout << "odometries size: " << vlo_odometries.size() << std::endl;
  assert(config_reader.odometries_parameteres_.size() != 0);
  /*******************************
           RUNNING PIPELINE
   *******************************/

  double initial_velocity = 0.0;
  auto pose1 = ground_truth_poses[config_reader.from_image];
  auto pose2 = ground_truth_poses[config_reader.from_image + 1];
  auto dt = config_reader.time_stamps[config_reader.from_image + 1] -
            config_reader.time_stamps[config_reader.from_image];

  auto relative_transform =
      vlo::Transform3D(pose1.matrix().inverse() * pose2.matrix());
  initial_velocity = relative_transform.translation().norm() / dt;
  auto initial_velocity_vector = relative_transform.translation() / dt;
  std::cout << "initial velocity from main: " << initial_velocity << std::endl;

  combo_vlo->sanity_checker_.previous_velocity_ = initial_velocity;
  combo_vlo->sanity_checker_.previous_velocity_vector_ =
      initial_velocity_vector;
  for (auto frame_id = config_reader.from_image;
       frame_id <
       config_reader.max_num_imgs_to_process + config_reader.from_image;
       frame_id += config_reader.frame_step) {
    //    if (!ros::ok()) {
    //      break;
    //    }

    std::cout
        << " ############################################################# "
        << std::endl;
    std::cout << "Running frame: " << frame_id << std::endl;
    std::cout << "Reading scan: " << config_reader.scans_path[frame_id]
              << std::endl;
    std::cout << "Reading image: " << config_reader.images_path[frame_id]
              << std::endl;
    std::cout << "Time stamp: " << config_reader.time_stamps[frame_id]
              << std::endl;
    std::cout
        << " ############################################################# "
        << std::endl;

    vlo::PointCloud current_scan;
    publish_velodyne(config_reader.scans_path[frame_id], current_scan);
    std::cout << "Before: " << current_scan.points_.size() << std::endl;

    std::cout << "After: " << current_scan.points_.size() << std::endl;
    //    current_scan.PaintUniformColor(vlo::Vec3(1.0, 0.0, 0.0));
    //    vlo::PointCloud copy(current_scan);
    //    copy = copy.Transform(camera_lidar_transform.matrix());
    //    open3d::io::WritePointCloud("gt" + std::to_string(frame_id) + ".pcd",
    //                                copy);
    cv::Mat image_rgb =
        imread(config_reader.images_path[frame_id], cv::IMREAD_COLOR);
    cv::imshow("image", image_rgb);
    waitKey(1);

    vlo::LidarFrame::Ptr lidar_frame =
        std::make_shared<vlo::LidarFrame>(current_scan, camera_lidar_transform);
    lidar_frame->point_cloud_.PaintUniformColor(vlo::Vec3(0.0, 1.0, 0.0));

    vlo::VisualFrame::Ptr camera_frame = std::make_shared<vlo::VisualFrame>(
        image_rgb, camera, config_reader.time_stamps[frame_id]);
    vlo::IFrame::Ptr vlo_frame = std::make_shared<vlo::VisualLidarFrame>(
        camera_frame, lidar_frame, camera_lidar_transform, color_pc_include);
    auto ground_truth_pose = ground_truth_poses[frame_id];
    Eigen::Matrix4d gt_transform = Eigen::Matrix4d::Identity();

    auto t_start = std::chrono::high_resolution_clock::now();

    combo_vlo->addVisualAndLidarFrames(vlo_frame);
    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed_time_ms =
        std::chrono::duration<double, std::milli>(t_end - t_start).count();
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
              << std::endl;
    std::cout << "Frame was running: " << elapsed_time_ms << " ms."
              << std::endl;

    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
              << std::endl;
    auto vlo_pose = combo_vlo->getLastPose();
    //    rviz_bridge_viewer->setCameraPose(vlo_pose);
    //    rviz_bridge_viewer->setGroundTruthPose(ground_truth_pose);

    if (config_reader.save) {
      saveData(vlo_pose, config_reader.dataset_number + "_combo");
      std::cout << "save index: " << combo_vlo->index_ << std::endl;
      std::ofstream myfile;
      myfile.open(config_reader.dataset_number + "_index_combo" + ".txt",
                  std::ios_base::app);
      myfile << combo_vlo->index_ << "\n";

      std::ofstream myfile3;
      myfile3.open(config_reader.dataset_number + "acceleration.txt",
                   std::ios_base::app);
      myfile3 << combo_vlo->velocity << "\n";
      myfile3.close();

      std::ofstream myfile2;
      myfile2.open(config_reader.dataset_number + "velocity.txt",
                   std::ios_base::app);
      myfile2 << combo_vlo->acceleration_ << "\n";
      myfile2.close();

      std::ofstream myfile4;
      myfile4.open(config_reader.dataset_number + "ackermann_velocity_.txt",
                   std::ios_base::app);
      myfile4 << combo_vlo->ackermann_velocity_ << "\n";
      myfile4.close();

      std::ofstream myfile10;
      myfile10.open("the_best_acc.txt", std::ios_base::app);
      myfile10 << combo_vlo->acceleration_vector_(0) << " "
               << combo_vlo->acceleration_vector_(1) << " "
               << combo_vlo->acceleration_vector_(2) << "\n";
      myfile10.close();

      myfile.close();
    }

    if (config_reader.from_image < frame_id) {
      gt_transform = ground_truth_poses[frame_id - 1].matrix().inverse() *
                     ground_truth_pose.matrix();
      std::cout << "In main: " << std::endl;
      std::cout << gt_transform.matrix() << std::endl;
      combo_vlo->gt_transform_ = gt_transform;
    }
  }
}
