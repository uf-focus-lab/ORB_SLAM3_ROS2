// =============================================================================
// Sensor message synchronization
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================

#pragma once

#include "threading/fifo.h"
#include "threading/leaky.h"
#include "types.h"

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.hpp>

class Sync {
private:
  const rclcpp::Node *node;
  IMUPipe imu_in;
  ImagePipe img_in;
  // Standalone thread for synchronization
  std::unique_ptr<std::thread> thread;
  bool flag_term = false;
  void loop();

  // Shared pointer for data output
  DataFramePipe *frame_out;

public:
  Sync(rclcpp::Node *node, DataFramePipe *frame_out);
  ~Sync();
  void handle_img_msg(msg::Image::SharedPtr img_msg);
  void handle_imu_msg(msg::IMU::SharedPtr imu_msg);
};
