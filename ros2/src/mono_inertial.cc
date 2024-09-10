// =============================================================================
// TODO: Add file description
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================

#include "common.h"
#include <sensor_msgs/image_encodings.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto slam = std::make_shared<SLAM>(ORB_SLAM3::System::eSensor::MONOCULAR);
  rclcpp::spin(slam);
  rclcpp::shutdown();
  return 0;
}
