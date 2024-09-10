// =============================================================================
// Sensor message synchronization
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================

#pragma once

#include "threading.h"
#include "types.h"

#include <memory>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.hpp>

class Sync {
private:
  const rclcpp::Node *node;
  threading::FIFO<msg::IMU::SharedPtr> imu_pipe;
  msg::Image *prev_img_msg = nullptr; // For comparison only, no ownership
  msg::Image::SharedPtr next_img_msg;
  msg::IMU::SharedPtr next_imu_msg;
  rclcpp::Time next_imu_time;
  msg::IMU::SharedPtr shift_imu();

  // Standalone thread for synchronization
  std::unique_ptr<std::thread> thread;
  bool flag_term = false;
  void loop();

  // Shared pointer for data output
  std::unique_ptr<DataFrame> *next_frame;

public:
  Sync(rclcpp::Node *node, std::unique_ptr<DataFrame> *next_frame);
  ~Sync();
  void handle_img_msg(msg::Image::SharedPtr img_msg);
  void handle_imu_msg(msg::IMU::SharedPtr imu_msg);
};
