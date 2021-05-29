#pragma once

#include <ros/ros.h>

#include <cassert>
#include <iostream>
#include <thread>
#include <vector>

#include "Laserscan.h"
#include "common_vlo.hpp"
#include "kitti_utils.h"
#include "pointmatcher/IO.h"
#include "pointmatcher/PointMatcher.h"
//////////////////////////
#include <open3d/Open3D.h>
#include <ros/ros.h>

#include <fstream>
#include <iostream>
#include <string>

#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "geometry/Camera.hpp"
#include "kitti_utils.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "yaml-cpp/yaml.h"

namespace fs = boost::filesystem;

using namespace std;

std::vector<vlo::Transform3D> read_poses(const std::string& poses_file) {
  return KITTI::Odometry::loadPosesToIsometry3D(poses_file);
}

bool checkInputArguments(int argc, char** argv) {
  // The only argument is Path to the configuration file, which stores the
  // dataset_dir and camera_info
  const int kNumArguments = 1;
  if (argc - 1 != kNumArguments) {
    cout << "Lack arguments: Please input the path to the .yaml config file"
         << endl;
    return false;
  }
  return true;
}

std::vector<std::string> readScanPaths(const string& dataset_dir,
                                       int num_images,
                                       const string& image_formatting,
                                       bool is_print_res) {
  // Set up image_paths
  vector<string> image_paths;
  boost::format filename_fmt(dataset_dir + image_formatting);
  for (int i = 0; i < num_images; i++) {
    image_paths.push_back((filename_fmt % i).str());
  }

  // Print result
  if (is_print_res) {
    cout << endl;
    cout << "Reading from dataset_dir: " << dataset_dir << endl;
    cout << "Number of images: " << image_paths.size() << endl;
    cout << "Print the first 5 image filenames:" << endl;
    for (auto s : image_paths)
      cout << s << endl;
    cout << endl;
  }
  return image_paths;
}

std::vector<std::string> readKAISTScanPaths(const string& dataset_dir,
                                            int num_images,
                                            const string& image_formatting,
                                            bool is_print_res) {
  // Set up image_paths
  std::cout << "Read data folder to vector<string>" << std::endl;
  std::vector<string> paths;

  for (const auto& entry : fs::directory_iterator(dataset_dir)) {
    paths.push_back(entry.path().string());
  }

  std::sort(paths.begin(), paths.end());

  // Print result
  if (is_print_res) {
    cout << endl;
    cout << "Reading from dataset_dir: " << dataset_dir << endl;
    cout << "Number of images: " << paths.size() << endl;
    cout << "Print the first 5 image filenames:" << endl;
    for (auto s : paths)
      cout << s << endl;
    cout << endl;
  }
  return paths;
}

std::vector<double> readTimeStamps(const std::string& dataset_file) {
  std::vector<double> time_stamps;

  std::ifstream infile;
  infile.open(dataset_file);

  if (infile.fail()) {
    cout << "Could not open file numbers."
         << "\n";
    exit(0);
  }

  double data;
  infile >> data;
  while (!infile.eof()) {
    time_stamps.push_back(data);
    infile >> data;
  }

  return time_stamps;
}

bool read_scan(std::string scan_name, open3d::geometry::PointCloud& scan) {
  std::ifstream in(scan_name.c_str(), std::ios::binary);
  if (!in.is_open())
    return false;

  //    scan.clear();

  in.seekg(0, std::ios::end);
  uint32_t num_points = in.tellg() / (4 * sizeof(float));
  in.seekg(0, std::ios::beg);

  std::vector<float> values(4 * num_points);
  in.read((char*)&values[0], 4 * num_points * sizeof(float));

  in.close();
  std::vector<vlo::Vec3>& points = scan.points_;
  //    std::vector<float>& remissions = scan.remissions_;

  points.resize(num_points);
  //    remissions.resize(num_points);

  float max_remission = 1.0; // assume that the remission is normalized.

  for (uint32_t i = 0; i < num_points; ++i) {
    points[i].x() = values[4 * i];
    points[i].y() = values[4 * i + 1];
    points[i].z() = values[4 * i + 2];
    //        remissions[i] = values[4 * i + 3];
    // max_remission = std::max(remissions[i], max_remission);
  }

  //    for (uint32_t i = 0; i < num_points; ++i) {
  //        remissions[i] /= max_remission;
  //    }

  return true;
}

struct OdometryParameters {
  std::string type;
  double sift_contrastThreshold;
  double sift_edgeThreshold;
  int sift_nOctaveLayers;
  int sift_nfeatures;
  double sift_sigma;
};

struct ConfigReader {
  std::string dataset_name, dataset_dir, image_folder, lidar_folder,
      ground_truth_data_file, icp_config_file, image_format, scan_format,
      feature_descriptor, time_stamps_file, image_dataset_dir,
      lidar_dataset_dir, dataset_number;
  int max_num_imgs_to_process, from_image, frame_step;
  bool debug, save, verbose, sanity_checker_on, ackermann_checker,
      dynamics_checker;
  std::vector<std::string> scans_path, images_path, scans_right_path,
      scans_left_path;
  std::vector<double> time_stamps;
  double r1, r2, r3, r4, r5, r6, r7, r8, r9, t1, t2, t3;
  double fx, fy, cx, cy;
  std::string sanity_checker_mode;
  // sanity checker params
  double max_acceleration, max_velocity, max_x_displacement, max_y_displacement;
  std::vector<std::string> activated_vlos;
  std::vector<OdometryParameters> odometries_parameteres_;
};

ConfigReader readConfigFromYamlFile(const std::string& config_file_path) {
  ConfigReader config_reader;
  YAML::Node config = YAML::LoadFile(config_file_path);
  config_reader.dataset_name = config["dataset_name"].as<std::string>();
  config_reader.dataset_number =
      config[config_reader.dataset_name]["dataset_number"].as<std::string>();
  config_reader.dataset_dir =
      config[config_reader.dataset_name]["dataset_dir"].as<std::string>();
  config_reader.dataset_dir =
      config_reader.dataset_dir + config_reader.dataset_number;

  config_reader.image_folder =
      config[config_reader.dataset_name]["image_folder"].as<std::string>();
  config_reader.lidar_folder =
      config[config_reader.dataset_name]["lidar_folder"].as<std::string>();
  config_reader.ground_truth_data_file =
      config_reader.dataset_dir + "/" + config_reader.dataset_number + ".txt";
  config_reader.icp_config_file =
      config[config_reader.dataset_name]["icp_config_file"].as<std::string>();
  config_reader.max_num_imgs_to_process =
      config["max_num_imgs_to_process"].as<int>();
  config_reader.from_image = config["from_image"].as<int>();
  config_reader.debug = config["debug"].as<bool>();
  config_reader.sanity_checker_on = config["sanity_checker_on"].as<bool>();

  config_reader.verbose = config["verbose"].as<bool>();
  config_reader.save = config["save"].as<bool>();

  config_reader.image_format =
      config[config_reader.dataset_name]["image_format"].as<std::string>();
  config_reader.scan_format =
      config[config_reader.dataset_name]["scan_format"].as<std::string>();
  config_reader.feature_descriptor =
      config["feature_descriptor"].as<std::string>();
  config_reader.frame_step = config["frame_step"].as<int>();
  config_reader.time_stamps_file = config_reader.dataset_dir + "/times.txt";
  config_reader.r1 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["r1"]
          .as<double>();
  config_reader.r2 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["r2"]
          .as<double>();
  config_reader.r3 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["r3"]
          .as<double>();
  config_reader.r4 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["r4"]
          .as<double>();
  config_reader.r5 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["r5"]
          .as<double>();
  config_reader.r6 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["r6"]
          .as<double>();
  config_reader.r7 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["r7"]
          .as<double>();
  config_reader.r8 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["r8"]
          .as<double>();
  config_reader.r9 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["r9"]
          .as<double>();
  config_reader.t1 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["t1"]
          .as<double>();
  config_reader.t2 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["t2"]
          .as<double>();
  config_reader.t3 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["t3"]
          .as<double>();

  config_reader.image_dataset_dir =
      config_reader.dataset_dir + "/" + config_reader.image_folder;
  config_reader.lidar_dataset_dir =
      config_reader.dataset_dir + "/" + config_reader.lidar_folder;

  std::cout << "Dataset name: " << config_reader.dataset_name << std::endl;
  std::cout << "Dataset number: " << config_reader.dataset_number << std::endl;
  std::cout << "Dataset dir: " << config_reader.dataset_dir << std::endl;
  std::cout << "Image path: " << config_reader.image_dataset_dir << std::endl;
  std::cout << "Lidar path: " << config_reader.lidar_dataset_dir << std::endl;
  std::cout << "Ground truth file: " << config_reader.ground_truth_data_file
            << std::endl;
  std::cout << "From image: " << config_reader.from_image << std::endl;
  std::cout << "Images to process: " << config_reader.max_num_imgs_to_process
            << std::endl;
  std::cout << "image_format: " << config_reader.image_format << std::endl;
  std::cout << "scan_format: " << config_reader.scan_format << std::endl;
  std::cout << "ICP config file: " << config_reader.icp_config_file
            << std::endl;
  std::cout << "Time stamps: " << config_reader.time_stamps_file << std::endl;
  std::cout << "Debug: " << config_reader.debug << std::endl;
  std::cout << "Verbose: " << config_reader.verbose << std::endl;
  std::cout << "Sanity checker state: " << config_reader.sanity_checker_on
            << std::endl;
  std::cout << "Reading scans. " << std::endl;
  config_reader.scans_path = readScanPaths(
      config_reader.lidar_dataset_dir,
      config_reader.from_image + config_reader.max_num_imgs_to_process,
      config_reader.scan_format,
      false);

  std::cout << "Reading images. " << std::endl;
  config_reader.images_path = readScanPaths(
      config_reader.image_dataset_dir,
      config_reader.from_image + config_reader.max_num_imgs_to_process,
      config_reader.image_format,
      false);
  std::cout << "Reading time stamps. " << std::endl;
  config_reader.time_stamps = readTimeStamps(config_reader.time_stamps_file);
  config_reader.fx =
      config[config_reader.dataset_name]["camera"]["fx"].as<double>();
  config_reader.fy =
      config[config_reader.dataset_name]["camera"]["fy"].as<double>();
  config_reader.cx =
      config[config_reader.dataset_name]["camera"]["cx"].as<double>();
  config_reader.cy =
      config[config_reader.dataset_name]["camera"]["cy"].as<double>();

  std::cout << "Camera parameters:" << std::endl;
  std::cout << "Camera fx: " << config_reader.fx << std::endl;
  std::cout << "Camera fy: " << config_reader.fy << std::endl;
  std::cout << "Camera cx: " << config_reader.cx << std::endl;
  std::cout << "Camera cy: " << config_reader.cy << std::endl;

  // sanity checker params

  config_reader.sanity_checker_mode =
      config["sanity_checker_mode"].as<std::string>();

  std::cout << "Sanity checker mode: " << config_reader.sanity_checker_mode
            << std::endl;

  config_reader.max_acceleration =
      config["sanity_checker"]["max_acceleration"].as<float>();
  config_reader.max_velocity =
      config["sanity_checker"]["max_velocity"].as<float>();
  config_reader.max_x_displacement =
      config["sanity_checker"]["max_x_displacement"].as<float>();
  config_reader.max_y_displacement =
      config["sanity_checker"]["max_y_displacement"].as<float>();
  config_reader.dynamics_checker =
      config["sanity_checker"]["dynamics_checker"].as<bool>();
  config_reader.ackermann_checker =
      config["sanity_checker"]["ackermann_checker"].as<bool>();

  std::cout << "Sanity checker: " << config_reader.sanity_checker_on
            << std::endl;
  if (config_reader.sanity_checker_on == true) {
    std::cout << "max_acceleration   " << config_reader.max_acceleration
              << std::endl;
    std::cout << "max_velocity " << config_reader.max_velocity << std::endl;
    std::cout << "max_x_displacement " << config_reader.max_x_displacement
              << std::endl;
    std::cout << "max_y_displacement " << config_reader.max_y_displacement
              << std::endl;
    std::cout << "Check dynamics: " << config_reader.dynamics_checker
              << std::endl;
    std::cout << "Ackermann checing: " << config_reader.ackermann_checker
              << std::endl;
  }

  auto odometries = config["odometries"];
  std::cout << "Number of odometries: " << odometries.size() << std::endl;

  for (const auto& odometry : config["odometries"]) {
    std::cout << "===========================================" << std::endl;
    if (odometry.first.as<std::string>() == "huang_vlo") {
      std::cout << "Huangs parameters " << std::endl;
      std::cout << "___________________________________________" << std::endl;
      OdometryParameters huangs_parameters;
      huangs_parameters.type = "huang_vlo";
      huangs_parameters.sift_contrastThreshold =
          odometry.second["sift"]["contrastThreshold"].as<double>();
      huangs_parameters.sift_edgeThreshold =
          odometry.second["sift"]["edgeThreshold"].as<double>();
      huangs_parameters.sift_nOctaveLayers =
          odometry.second["sift"]["nOctaveLayers"].as<int>();
      huangs_parameters.sift_nfeatures =
          odometry.second["sift"]["nfeatures"].as<int>();
      huangs_parameters.sift_sigma =
          odometry.second["sift"]["sigma"].as<double>();
      config_reader.odometries_parameteres_.push_back(huangs_parameters);
      std::cout << "SIFT parameters: " << std::endl;
      std::cout << "contrastThreshold "
                << huangs_parameters.sift_contrastThreshold << std::endl;
      std::cout << "edgeThreshold     " << huangs_parameters.sift_edgeThreshold
                << std::endl;
      std::cout << "nOctaveLayers     " << huangs_parameters.sift_nOctaveLayers
                << std::endl;
      std::cout << "nfeatures         " << huangs_parameters.sift_nfeatures
                << std::endl;
      std::cout << "sigma             " << huangs_parameters.sift_sigma
                << std::endl;
    }
    if (odometry.first.as<std::string>() == "point2plane_lo") {
      std::cout << "Point to plane parameters" << std::endl;
      std::cout << "___________________________________________" << std::endl;
      OdometryParameters point2plane_lo;
      point2plane_lo.type = "point2plane_lo";
      config_reader.odometries_parameteres_.push_back(point2plane_lo);
    }
    if (odometry.first.as<std::string>() == "colorbased_vlo") {
      std::cout << "ColorBased VLO parameters" << std::endl;
      std::cout << "___________________________________________" << std::endl;
      OdometryParameters colorBased_vlo_params;
      colorBased_vlo_params.type = "colorbased_vlo";
      config_reader.odometries_parameteres_.push_back(colorBased_vlo_params);
    }
    if (odometry.first.as<std::string>() == "ndt_lo") {
      std::cout << "NDT LO parameters" << std::endl;
      std::cout << "___________________________________________" << std::endl;
      OdometryParameters ndt_lo;
      ndt_lo.type = "ndt_lo";
      config_reader.odometries_parameteres_.push_back(ndt_lo);
    }
    if (odometry.first.as<std::string>() == "gicp_lo") {
      std::cout << "GICP LO parameters" << std::endl;
      std::cout << "___________________________________________" << std::endl;
      OdometryParameters gicp_lo;
      gicp_lo.type = "gicp_lo";
      config_reader.odometries_parameteres_.push_back(gicp_lo);
    }
  }
  std::cout << "===========================================" << std::endl;

  return config_reader;
}

ConfigReader readKAISTConfigFromYamlFile(const std::string& config_file_path) {
  ConfigReader config_reader;
  YAML::Node config = YAML::LoadFile(config_file_path);
  config_reader.dataset_name = config["dataset_name"].as<std::string>();
  config_reader.dataset_number =
      config[config_reader.dataset_name]["dataset_number"].as<std::string>();
  config_reader.dataset_dir =
      config[config_reader.dataset_name]["dataset_dir"].as<std::string>();
  config_reader.dataset_dir =
      config_reader.dataset_dir + config_reader.dataset_number;

  config_reader.image_folder =
      config[config_reader.dataset_name]["image_folder"].as<std::string>();
  config_reader.lidar_folder =
      config[config_reader.dataset_name]["lidar_folder"].as<std::string>();
  config_reader.ground_truth_data_file =
      config_reader.dataset_dir + "/" + config_reader.dataset_number + ".csv";
  config_reader.icp_config_file =
      config[config_reader.dataset_name]["icp_config_file"].as<std::string>();
  config_reader.max_num_imgs_to_process =
      config["max_num_imgs_to_process"].as<int>();
  config_reader.from_image = config["from_image"].as<int>();
  config_reader.debug = config["debug"].as<bool>();
  config_reader.sanity_checker_on = config["sanity_checker_on"].as<bool>();

  config_reader.verbose = config["verbose"].as<bool>();
  config_reader.save = config["save"].as<bool>();

  config_reader.image_format =
      config[config_reader.dataset_name]["image_format"].as<std::string>();
  config_reader.scan_format =
      config[config_reader.dataset_name]["scan_format"].as<std::string>();
  config_reader.feature_descriptor =
      config["feature_descriptor"].as<std::string>();
  config_reader.frame_step = config["frame_step"].as<int>();
  config_reader.time_stamps_file = config_reader.dataset_dir + "/times.csv";
  config_reader.r1 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["r1"]
          .as<double>();
  config_reader.r2 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["r2"]
          .as<double>();
  config_reader.r3 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["r3"]
          .as<double>();
  config_reader.r4 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["r4"]
          .as<double>();
  config_reader.r5 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["r5"]
          .as<double>();
  config_reader.r6 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["r6"]
          .as<double>();
  config_reader.r7 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["r7"]
          .as<double>();
  config_reader.r8 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["r8"]
          .as<double>();
  config_reader.r9 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["r9"]
          .as<double>();
  config_reader.t1 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["t1"]
          .as<double>();
  config_reader.t2 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["t2"]
          .as<double>();
  config_reader.t3 =
      config[config_reader.dataset_name]["camera_lidar_transform"]["t3"]
          .as<double>();

  config_reader.image_dataset_dir =
      config_reader.dataset_dir + "/" + config_reader.image_folder;
  config_reader.lidar_dataset_dir =
      config_reader.dataset_dir + "/" + config_reader.lidar_folder;

  std::cout << "Dataset name: " << config_reader.dataset_name << std::endl;
  std::cout << "Dataset number: " << config_reader.dataset_number << std::endl;
  std::cout << "Dataset dir: " << config_reader.dataset_dir << std::endl;
  std::cout << "Image path: " << config_reader.image_dataset_dir << std::endl;
  std::cout << "Lidar path: " << config_reader.lidar_dataset_dir << std::endl;
  std::cout << "Ground truth file: " << config_reader.ground_truth_data_file
            << std::endl;
  std::cout << "From image: " << config_reader.from_image << std::endl;
  std::cout << "Images to process: " << config_reader.max_num_imgs_to_process
            << std::endl;
  std::cout << "image_format: " << config_reader.image_format << std::endl;
  std::cout << "scan_format: " << config_reader.scan_format << std::endl;
  std::cout << "ICP config file: " << config_reader.icp_config_file
            << std::endl;
  std::cout << "Time stamps: " << config_reader.time_stamps_file << std::endl;
  std::cout << "Debug: " << config_reader.debug << std::endl;
  std::cout << "Verbose: " << config_reader.verbose << std::endl;
  std::cout << "Sanity checker state: " << config_reader.sanity_checker_on
            << std::endl;
  std::cout << "Reading scans. " << std::endl;
  config_reader.scans_left_path = readKAISTScanPaths(
      config_reader.lidar_dataset_dir + "/VLP_left/",
      config_reader.from_image + config_reader.max_num_imgs_to_process,
      config_reader.scan_format,
      false);

  config_reader.scans_right_path = readKAISTScanPaths(
      config_reader.lidar_dataset_dir + "/VLP_right/",
      config_reader.from_image + config_reader.max_num_imgs_to_process,
      config_reader.scan_format,
      false);

  std::cout << "Reading images. " << std::endl;
  config_reader.images_path = readKAISTScanPaths(
      config_reader.image_dataset_dir,
      config_reader.from_image + config_reader.max_num_imgs_to_process,
      config_reader.image_format,
      false);
  std::cout << "Reading time stamps. " << std::endl;
  config_reader.time_stamps = readTimeStamps(config_reader.time_stamps_file);
  config_reader.fx =
      config[config_reader.dataset_name]["camera"]["fx"].as<double>();
  config_reader.fy =
      config[config_reader.dataset_name]["camera"]["fy"].as<double>();
  config_reader.cx =
      config[config_reader.dataset_name]["camera"]["cx"].as<double>();
  config_reader.cy =
      config[config_reader.dataset_name]["camera"]["cy"].as<double>();

  std::cout << "Camera parameters:" << std::endl;
  std::cout << "Camera fx: " << config_reader.fx << std::endl;
  std::cout << "Camera fy: " << config_reader.fy << std::endl;
  std::cout << "Camera cx: " << config_reader.cx << std::endl;
  std::cout << "Camera cy: " << config_reader.cy << std::endl;

  // sanity checker params

  config_reader.sanity_checker_mode =
      config["sanity_checker_mode"].as<std::string>();

  std::cout << "Sanity checker mode: " << config_reader.sanity_checker_mode
            << std::endl;

  config_reader.max_acceleration =
      config["sanity_checker"]["max_acceleration"].as<float>();
  config_reader.max_velocity =
      config["sanity_checker"]["max_velocity"].as<float>();
  config_reader.max_x_displacement =
      config["sanity_checker"]["max_x_displacement"].as<float>();
  config_reader.max_y_displacement =
      config["sanity_checker"]["max_y_displacement"].as<float>();
  config_reader.dynamics_checker =
      config["sanity_checker"]["dynamics_checker"].as<bool>();
  config_reader.ackermann_checker =
      config["sanity_checker"]["ackermann_checker"].as<bool>();

  std::cout << "Sanity checker: " << config_reader.sanity_checker_on
            << std::endl;
  if (config_reader.sanity_checker_on == true) {
    std::cout << "max_acceleration   " << config_reader.max_acceleration
              << std::endl;
    std::cout << "max_velocity " << config_reader.max_velocity << std::endl;
    std::cout << "max_x_displacement " << config_reader.max_x_displacement
              << std::endl;
    std::cout << "max_y_displacement " << config_reader.max_y_displacement
              << std::endl;
    std::cout << "Check dynamics: " << config_reader.dynamics_checker
              << std::endl;
    std::cout << "Ackermann checing: " << config_reader.ackermann_checker
              << std::endl;
  }

  auto odometries = config["odometries"];
  std::cout << "Number of odometries: " << odometries.size() << std::endl;

  for (const auto& odometry : config["odometries"]) {
    std::cout << "===========================================" << std::endl;
    if (odometry.first.as<std::string>() == "huang_vlo") {
      std::cout << "Huangs parameters " << std::endl;
      std::cout << "___________________________________________" << std::endl;
      OdometryParameters huangs_parameters;
      huangs_parameters.type = "huang_vlo";
      huangs_parameters.sift_contrastThreshold =
          odometry.second["sift"]["contrastThreshold"].as<double>();
      huangs_parameters.sift_edgeThreshold =
          odometry.second["sift"]["edgeThreshold"].as<double>();
      huangs_parameters.sift_nOctaveLayers =
          odometry.second["sift"]["nOctaveLayers"].as<int>();
      huangs_parameters.sift_nfeatures =
          odometry.second["sift"]["nfeatures"].as<int>();
      huangs_parameters.sift_sigma =
          odometry.second["sift"]["sigma"].as<double>();
      config_reader.odometries_parameteres_.push_back(huangs_parameters);
      std::cout << "SIFT parameters: " << std::endl;
      std::cout << "contrastThreshold "
                << huangs_parameters.sift_contrastThreshold << std::endl;
      std::cout << "edgeThreshold     " << huangs_parameters.sift_edgeThreshold
                << std::endl;
      std::cout << "nOctaveLayers     " << huangs_parameters.sift_nOctaveLayers
                << std::endl;
      std::cout << "nfeatures         " << huangs_parameters.sift_nfeatures
                << std::endl;
      std::cout << "sigma             " << huangs_parameters.sift_sigma
                << std::endl;
    }
    if (odometry.first.as<std::string>() == "point2plane_lo") {
      std::cout << "Point to plane parameters" << std::endl;
      std::cout << "___________________________________________" << std::endl;
      OdometryParameters point2plane_lo;
      point2plane_lo.type = "point2plane_lo";
      config_reader.odometries_parameteres_.push_back(point2plane_lo);
    }
    if (odometry.first.as<std::string>() == "colorbased_vlo") {
      std::cout << "ColorBased VLO parameters" << std::endl;
      std::cout << "___________________________________________" << std::endl;
      OdometryParameters colorBased_vlo_params;
      colorBased_vlo_params.type = "colorbased_vlo";
      config_reader.odometries_parameteres_.push_back(colorBased_vlo_params);
    }
    if (odometry.first.as<std::string>() == "ndt_lo") {
      std::cout << "NDT LO parameters" << std::endl;
      std::cout << "___________________________________________" << std::endl;
      OdometryParameters ndt_lo;
      ndt_lo.type = "ndt_lo";
      config_reader.odometries_parameteres_.push_back(ndt_lo);
    }
    if (odometry.first.as<std::string>() == "gicp_lo") {
      std::cout << "GICP LO parameters" << std::endl;
      std::cout << "___________________________________________" << std::endl;
      OdometryParameters gicp_lo;
      gicp_lo.type = "gicp_lo";
      config_reader.odometries_parameteres_.push_back(gicp_lo);
    }
  }
  std::cout << "===========================================" << std::endl;

  return config_reader;
}

void saveData(const vlo::Transform3D vlo_pose,
              std::string file_name = "output") {
  const static Eigen::IOFormat CommaInitFmt(
      Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ", "", "", "");
  ofstream myfile;
  myfile.open(file_name + ".txt", std::ios_base::app);
  myfile << vlo_pose.matrix().block<3, 4>(0, 0).format(CommaInitFmt) << '\n';
  myfile.close();
}

void publish_velodyne(string infile, vlo::PointCloud& point_cloud) {
  fstream input(infile.c_str(), ios::in | ios::binary);
  if (!input.good()) {
    ROS_ERROR_STREAM("Could not read file: " << infile);
  } else {
    ROS_DEBUG_STREAM("reading " << infile);
    input.seekg(0, ios::beg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr points(
        new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<vlo::Vec3> eigen_points;

    int i;
    for (i = 0; input.good() && !input.eof(); i++) {
      pcl::PointXYZI point;
      input.read((char*)&point.x, 3 * sizeof(float));
      input.read((char*)&point.intensity, sizeof(float));

      vlo::Vec3 eigen_point(point.x, point.y, point.z);
      point_cloud.points_.push_back(eigen_point);
      point_cloud.colors_.push_back(
          vlo::Vec3(point.intensity, point.intensity, point.intensity));
    }

    input.close();
  }
}
