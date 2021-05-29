#include "SanityChecker/SanityChecker.hpp"

#include <fstream>
#include <iostream>

#include <opencv2/calib3d/calib3d.hpp>
namespace vlo {
SanityChecker::SanityChecker() {}

bool SanityChecker::dynamicsChecker(
    const IFrame::Ptr &current_frame,
    std::vector<IFrame::Ptr> &nodes_trajectory) const {
  if (nodes_trajectory.size() > 0) {
    auto delta_time =
        current_frame->time_stamp_ - nodes_trajectory.back()->time_stamp_;
    auto frame_to_frame_transform =
        nodes_trajectory.back()->pose_.inverse() * current_frame->pose_;
    auto speed = frame_to_frame_transform.translation().norm() / delta_time;
    current_frame->speed = speed;
    current_frame->acceleration =
        (current_frame->speed - nodes_trajectory.back()->speed) / delta_time;
    if (std::abs(current_frame->acceleration) > max_acceleration_) {
      return true;
    } else {
      return false;
    }
  }
  std::cout << " Not enough frame " << std::endl;
  return false;
}

std::tuple<double, double, bool, vlo::Vec3> SanityChecker::dynamicsChecker(
    const Transform3D &proposal_transform, const IFrame::Ptr &current_frame,
    const IFrame::Ptr &reference_frame) {
  vlo::Vec3 current_velocity_vector =  (proposal_transform.translation()/
                             (current_frame->time_stamp_ - reference_frame->time_stamp_));
  auto current_velocity =
      (proposal_transform.translation().norm() /
       (current_frame->time_stamp_ - reference_frame->time_stamp_));
  auto acceleration =
      (current_velocity - previous_velocity_) /
      (current_frame->time_stamp_ - reference_frame->time_stamp_);

  auto acceleration_vector = (current_velocity_vector - previous_velocity_vector_)/
      (current_frame->time_stamp_ - reference_frame->time_stamp_);


  std::cout << "velocity : " << current_velocity << std::endl;
  std::cout << "acceleration : " << acceleration << std::endl;
  std::cout << "acceleration vector: " << acceleration_vector << std::endl;

  std::ofstream myfile3;
  myfile3.open( "all_accelerations.txt", std::ios_base::app);
  myfile3 <<acceleration_vector(0) << " " << acceleration_vector(1) << " " << acceleration_vector(2)  << "\n";
  myfile3.close();

  if (std::abs(acceleration) > max_acceleration_) {
    return std::tuple<double, double, bool,vlo::Vec3>(acceleration, current_velocity,
                                            true, current_velocity_vector);
  } else {
    return std::tuple<double, double, bool, vlo::Vec3>(acceleration, current_velocity,
                                            false, current_velocity_vector);
  }
}

bool SanityChecker::trustResult(
    const IFrame::Ptr &current_frame,
    std::vector<IFrame::Ptr> &nodes_trajectory) const {
  std::cout << "Activated? " << activated_ << std::endl;
  bool trust_result = true;
  if (activated_) {
    if (check_dynamics_) {
      std::cout << "Dynamics checking ... " << std::endl;
      bool trust_dynamics = dynamicsChecker(current_frame, nodes_trajectory);
      trust_result = trust_dynamics and trust_result;
    }
    if (check_ackermann_) {
      std::cout << "Ackermann checking ... " << std::endl;
      auto frame_to_frame_transform =
          nodes_trajectory.back()->pose_.inverse() * current_frame->pose_;
      bool trust_ackermann = checkAckermann(
          frame_to_frame_transform,
          current_frame->time_stamp_ - nodes_trajectory.back()->time_stamp_);
      trust_result = trust_ackermann and trust_result;
    }
  }
  return trust_result;
}

std::tuple<double, bool> SanityChecker::checkAckermann(
    const Transform3D &frame_to_frame_transform,
    const IFrame::Ptr &current_frame, const IFrame::Ptr &reference_frame) {
  std::cout << "Checking Ackermann." << std::endl;
  auto dt = current_frame->time_stamp_ - reference_frame->time_stamp_;
  auto f = 1.0 / dt;
  auto Vx = f * frame_to_frame_transform.translation()(0);
  auto Vz = f * frame_to_frame_transform.translation()(2);

  auto yaw = frame_to_frame_transform.rotation().eulerAngles(0, 1, 2)(1);
  if (frame_to_frame_transform.rotation().eulerAngles(0, 1, 2)(1) >
      M_PI / 2.0) {
    yaw = M_PI - yaw;
  }
  if (frame_to_frame_transform.rotation().eulerAngles(0, 1, 2)(1) <
      -M_PI / 2.0) {
    yaw = -(M_PI + yaw);
  }

  double l = 1.03;
  double Vx_estimated = Vx;
//  if (std::abs(yaw) >1.0)
//  {
  if (yaw == 0.0) {
    std::cout << "Vx: " << Vx << " Vz: " << Vz << "yaw: " << yaw
              << " Vxestimated: " << 0.0 << std::endl;
    Vx = 0.0;
  } else {
    Vx_estimated =
        f * ((Vz / f) * l * (1 - cos(yaw))) / sin(yaw) * (1 - cos(yaw)) +
        l * sin(yaw);

    std::cout << "Vx: " << Vx << " Vz: " << Vz << "yaw: " << yaw
              << " Vxestimated: " << Vx_estimated << std::endl;
  }
  std::cout << " Ackermann error: " << std::abs(Vx - Vx_estimated) << std::endl;

  std::ofstream myfile3;
  myfile3.open( "achkermann.txt", std::ios_base::app);
  myfile3 << Vx - Vx_estimated << "\n";
  myfile3.close();

  if (std::abs(Vx - Vx_estimated) > 0.7)
  {
    return std::tuple<double, bool> (std::abs(Vx - Vx_estimated), true);
  }
  else {
      return std::tuple<double, bool> (std::abs(Vx - Vx_estimated), false);
  }
//  }
//  else {

//      return 1.0;}
}

bool SanityChecker::checkAckermann(Transform3D &frame_to_frame_transform,
                                   double dt) const {
  auto yaw = frame_to_frame_transform.rotation().eulerAngles(0, 1, 2)(1);
  if (frame_to_frame_transform.rotation().eulerAngles(0, 1, 2)(1) >
      M_PI / 2.0) {
    yaw = M_PI - yaw;
  }
  if (frame_to_frame_transform.rotation().eulerAngles(0, 1, 2)(1) <
      -M_PI / 2.0) {
    yaw = -(M_PI + yaw);
  }
  auto x = frame_to_frame_transform.translation()(0);
  auto R = frame_to_frame_transform.translation()(2) / std::sin(yaw);
  auto constraint = std::pow(x, 2) - 2 * R * x +
                    std::pow(frame_to_frame_transform.translation()(2), 2);
  frame_to_frame_transform.translation()(0) =
      (frame_to_frame_transform.translation()(2) / std::sin(yaw)) *
      (1 - std::cos(yaw));
  return false;
}

bool SanityChecker::checkAckermann(const IFrame::Ptr &current_frame,
                                   const IFrame::Ptr &reference_frame) const {}

}  // namespace vlo
