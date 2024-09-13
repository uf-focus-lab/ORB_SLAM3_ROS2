// =============================================================================
// TODO: Add file description
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================

#include "common.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto slam = std::make_shared<SLAM>(ORB_SLAM3::System::eSensor::IMU_MONOCULAR);
  rclcpp::spin(slam);
  rclcpp::shutdown();
  return 0;
}
