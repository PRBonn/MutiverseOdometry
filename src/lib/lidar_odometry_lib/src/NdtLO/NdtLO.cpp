#include "NdtLO/NdtLO.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <ros/ros.h>

#include <iostream>
//#include <pcl/visualization/pcl_visualizer.h>
#include <NdtLO/pclomp/gicp_omp.h>
#include <NdtLO/pclomp/ndt_omp.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "open3d_conversions/open3d_conversions.h"

namespace vlo {
NdtLO::NdtLO(const vlo::Transform3D &initial_pose,
             const std::string &config_file)
    : initial_pose_(initial_pose){
          //  std::ifstream ifs(config_file.c_str());
          //  icp.loadFromYaml(ifs);
      };

void NdtLO::processFrame(IFrame::Ptr lidar_frame) {
  current_lidar_frame_ = lidar_frame;

  if (lo_state_ == VLO_STATE::INIT) {
    current_lidar_frame_->pose_ = initial_pose_;
    pushFrameToHistory(current_lidar_frame_);
    lo_state_ = VLO_STATE::RUNNING;
  } else if (lo_state_ == VLO_STATE::RUNNING) {
    auto [frame_to_frame_transform, trust_result] =
        findFrameToFrameTransformBetween(current_lidar_frame_,
                                         reference_lidar_frame_);
    current_lidar_frame_->pose_.matrix() =
        reference_lidar_frame_->pose_.matrix() *
        frame_to_frame_transform.matrix();
    scale_ = frame_to_frame_transform.translation();

    pushFrameToHistory(current_lidar_frame_);
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr align(
    pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud,
    const Mat44f &last_transform = Mat44f::Identity()) {
  registration->setInputTarget(target_cloud);
  registration->setInputSource(source_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(
      new pcl::PointCloud<pcl::PointXYZ>());

  auto t1 = ros::WallTime::now();
  registration->align(*aligned, last_transform);
  auto t2 = ros::WallTime::now();
  std::cout << "single : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;

  //  for (int i = 0; i < 10; i++) {
  //    registration->align(*aligned);
  //  }
  auto t3 = ros::WallTime::now();
  std::cout << "10times: " << (t3 - t2).toSec() * 1000 << "[msec]" << std::endl;
  std::cout << "fitness: " << registration->getFitnessScore() << std::endl
            << std::endl;

  return aligned;
}

std::tuple<Transform3D, bool> NdtLO::findFrameToFrameTransformBetween(
    IFrame::Ptr &current_frame, IFrame::Ptr &reference_frame) {
  std::cout << "NdtLO looks for the transformation between "
               "current_frame and reference_frame"
            << std::endl;
  std::cout << "NdtLO: Last transform: " << std::endl;
  std::cout << last_transform_.matrix() << std::endl;

  //  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

  sensor_msgs::PointCloud2 current_pc_temp;
  sensor_msgs::PointCloud2 reference_pc_temp;

  open3d_conversions::open3dToRos(current_frame->point_cloud_, current_pc_temp,
                                  "world");
  open3d_conversions::open3dToRos(reference_frame->point_cloud_,
                                  reference_pc_temp, "world");

  pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::moveFromROSMsg<pcl::PointXYZ>(current_pc_temp, *current_cloud);
  pcl::moveFromROSMsg<pcl::PointXYZ>(reference_pc_temp, *reference_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  target_cloud = reference_cloud;
  source_cloud = current_cloud;

  //  if (pcl::io::loadPCDFile<pcl::PointXYZ>
  //  ("/home/andrzej/code_workspace/lol/src/ndt_omp/data/251370668.pcd",
  //  *target_cloud) == -1) //* load the file
  //  {
  //      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
  //      exit(EXIT_FAILURE);
  //  }

  //  if (pcl::io::loadPCDFile<pcl::PointXYZ>
  //  ("/home/andrzej/code_workspace/lol/src/ndt_omp/data/251371071.pcd",
  //  *source_cloud) == -1) //* load the file
  //  {
  //      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
  //      exit(EXIT_FAILURE);
  //  }

  // downsampling
  //  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(
  //      new pcl::PointCloud<pcl::PointXYZ>());

  //  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
  //  voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);

  //  voxelgrid.setInputCloud(target_cloud);
  //  voxelgrid.filter(*downsampled);
  //  *target_cloud = *downsampled;

  //  voxelgrid.setInputCloud(source_cloud);
  //  voxelgrid.filter(*downsampled);
  //  source_cloud = downsampled;

  ros::Time::init();

  std::cout << "target pcs : " << target_cloud->size()
            << " source pcs: " << source_cloud->size() << std::endl;
  //  // benchmark
  //  std::cout << "--- pcl::GICP ---" << std::endl;
  //  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr
  //  gicp(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ,
  //  pcl::PointXYZ>()); pcl::PointCloud<pcl::PointXYZ>::Ptr aligned =
  //  align(gicp, target_cloud, source_cloud);

  //  std::cout << "--- pclomp::GICP ---" << std::endl;
  //  pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ,
  //  pcl::PointXYZ>::Ptr gicp_omp(new
  //  pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
  //   aligned = align(gicp_omp, target_cloud, source_cloud);

  //  std::cout << "--- pcl::NDT ---" << std::endl;
  //  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr
  //  ndt(new pcl::NormalDistributionsTransform<pcl::PointXYZ,
  //  pcl::PointXYZ>()); ndt->setResolution(1.0); aligned = align(ndt,
  //  target_cloud, source_cloud);

  std::vector<std::pair<std::string, pclomp::NeighborSearchMethod>>
      search_methods = {      {"KDTREE", pclomp::KDTREE},
                        //   {"DIRECT7", pclomp::DIRECT7},
                       /* {"DIRECT1", pclomp::DIRECT1}*/};

  pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr
      ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ,
                                                       pcl::PointXYZ>());
  ndt_omp->setResolution(0.1);

  ndt_omp->setTransformationEpsilon(0.01);
  //  // Setting maximum step size for More-Thuente line search.
  ndt_omp->setStepSize(0.1);
  //  // Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt_omp->setResolution(1.0);

  //  // Setting max number of registration iterations.
  ndt_omp->setMaximumIterations(35);

  //  for(int n : num_threads) {
  int n = omp_get_max_threads();
  for (const auto &search_method : search_methods) {
    std::cout << "--- pclomp::NDT (" << search_method.first << ", " << n
              << " threads) ---" << std::endl;
    ndt_omp->setNumThreads(n);
    ndt_omp->setNeighborhoodSearchMethod(search_method.second);
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned =
        align(ndt_omp, target_cloud, source_cloud,
              last_transform_.matrix().cast<float>());
    //      }
  }

  Transform3D final_transform = Transform3D::Identity();
  final_transform.matrix() =
      ndt_omp->getFinalTransformation().matrix().cast<double>();

  last_transform_ = final_transform.matrix().cast<double>();

  return {final_transform, true};
}

std::tuple<Transform3D, bool> NdtLO::findFrameToFrameTransformBetween(
    IFrame::Ptr &current_frame, PointCloud &map) {
  std::cout << "Point2Plane ICP looks for the transformation between "
               "current_frame and map"
            << std::endl;
  std::cout << "Point2Plane ICP: Last transform: " << std::endl;

  return {Transform3D::Identity(), true};
}

double NdtLO::calculateCrossCheckingError(const IFrame::Ptr &current_frame,
                                          const IFrame::Ptr &reference_frame) {
  // TODO:implement
  return 5.0;
}

void NdtLO::hintForICP(const Transform3D &guess_for_icp) {
  guess_for_icp_ = guess_for_icp;
}

OdometerStruct getNdtLOAndInitialize(Transform3D reference_pose,
                                     std::string config_file, bool debug,
                                     bool verbose) {
  std::shared_ptr<vlo::NdtLO> point2plane_lo =
      std::make_shared<vlo::NdtLO>(reference_pose, config_file);
  point2plane_lo->verbose_ = debug;
  point2plane_lo->debug_ = verbose;

  return vlo::OdometerStruct{point2plane_lo, Transform3D::Identity()};
}

};  // namespace vlo
