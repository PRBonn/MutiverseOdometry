/**
* @author Xieyuanli Chen
* @date 10.04.2020
* @brief use octomap to generate a map with given scans and poses used for global localization
*/


#include <boost/filesystem.hpp>

#include "kitti_utils.h"
#include "Laserscan.h"

using namespace rv;
using namespace std;


typedef boost::filesystem::directory_iterator diter;

std::vector<std::string> read_scans(const std::string& directory) {
  std::vector<std::string> listing;
  for (diter it = diter(directory); it != diter(); ++it)
    listing.push_back(it->path().string());

  std::sort(listing.begin(), listing.end());

  return listing;
}

bool read_scan(std::string scan_name, Laserscan& scan) {

  std::ifstream in(scan_name.c_str(), std::ios::binary);
  if (!in.is_open()) return false;

  scan.clear();

  in.seekg(0, std::ios::end);
  uint32_t num_points = in.tellg() / (4 * sizeof(float));
  in.seekg(0, std::ios::beg);

  std::vector<float> values(4 * num_points);
  in.read((char*)&values[0], 4 * num_points * sizeof(float));

  in.close();
  std::vector<Point3f>& points = scan.points_;
  std::vector<float>& remissions = scan.remissions_;

  points.resize(num_points);
  remissions.resize(num_points);

  float max_remission = 1.0; // assume that the remission is normalized.

  for (uint32_t i = 0; i < num_points; ++i) {
    points[i].x() = values[4 * i];
    points[i].y() = values[4 * i + 1];
    points[i].z() = values[4 * i + 2];
    remissions[i] = values[4 * i + 3];
    // max_remission = std::max(remissions[i], max_remission);
  }

  for (uint32_t i = 0; i < num_points; ++i) {
    remissions[i] /= max_remission;
  }

  return true;
}


std::vector<Eigen::Matrix4f> read_poses(const std::string& poses_file) {
  return  KITTI::Odometry::loadPoses(poses_file);
}

Eigen::Matrix4f read_calib(const std::string& calib_file, bool kitti_calib) {
  Eigen::Matrix4f Tr = Eigen::Matrix4f::Identity();
  if (kitti_calib){
    KITTICalibration calib_ = KITTICalibration(calib_file);
    Tr = calib_["Tr"];
  }
  return Tr;
}


int main(int argc, char** argv) {

  std::string scan_folder = "../../toy_data";
  std::string pose_file = "../../toy_data/00.txt";

  std::vector<std::string> scan_names = read_scans(scan_folder);
  std::vector<Eigen::Matrix4f> poses = read_poses(pose_file);

  std::string current_scan_path = scan_names[0];
  Laserscan current_scan;

  read_scan(current_scan_path, current_scan);

  for (auto point : current_scan.points_) {
     std::cout << point << std::endl;
  }

}
