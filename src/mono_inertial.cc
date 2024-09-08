#include "common.h"
#include <sensor_msgs/image_encodings.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  slam = std::make_shared<SLAM>(ORB_SLAM3::System::eSensor::MONOCULAR,
                                sensor_msgs::image_encodings::BGR8);
  rclcpp::spin(slam);
  rclcpp::shutdown();
  return 0;
}
