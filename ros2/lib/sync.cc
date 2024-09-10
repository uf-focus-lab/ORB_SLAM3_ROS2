// =============================================================================
// TODO: Add file description
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================

#include "sync.h"
#include "types.h"

using namespace std::chrono_literals;

Sync::Sync(rclcpp::Node *node, std::unique_ptr<DataFrame> *next_frame)
    : node(node), next_frame(next_frame) {
  thread = std::make_unique<std::thread>(&Sync::loop, this);
}

Sync::~Sync() {
  flag_term = true;
  thread->join();
}

void Sync::handle_img_msg(msg::Image::SharedPtr img_msg) {
  next_img_msg = img_msg;
};

void Sync::handle_imu_msg(msg::IMU::SharedPtr imu_msg) {
  imu_pipe.write(imu_msg);
};

ORB_SLAM3::IMU::Point imu_point(msg::IMU::SharedPtr imu_msg) {
  if (!imu_msg)
    throw std::runtime_error("NULL pointer in IMU pipe");
  rclcpp::Time time(imu_msg->header.stamp);
  const auto &acc = imu_msg->linear_acceleration;
  const auto &ang = imu_msg->angular_velocity;
  return {static_cast<float>(acc.x),
          static_cast<float>(acc.y),
          static_cast<float>(acc.z),
          static_cast<float>(ang.x),
          static_cast<float>(ang.y),
          static_cast<float>(ang.z),
          time.seconds()};
}

msg::IMU::SharedPtr Sync::shift_imu() {
  auto prev_imu_msg = next_imu_msg;
  next_imu_msg = imu_pipe.read();
  next_imu_time = next_imu_msg->header.stamp;
  return prev_imu_msg;
}

void Sync::loop() {
  try {
    // Load first IMU message
    shift_imu();
    while (!flag_term) {
      std::this_thread::sleep_for(1ms);
      // Only look once
      auto img_msg = next_img_msg;
      // Wait for next image message
      if (prev_img_msg == img_msg.get() || !img_msg)
        continue;
      rclcpp::Time img_time(img_msg->header.stamp);
      auto frame = std::make_unique<DataFrame>();
      frame->time = img_time;
      // Stack all IMU messages before current image taken
      while (next_imu_time <= img_time) {
        const auto point = imu_point(shift_imu());
        frame->imu.push_back(point);
        frame->Wbb << point.w.x(), point.w.y(), point.w.z();
      }
      // Synchronize IMU and Image
      try {
        // ORB_SLAM3 will convert all images to grayscale (U8).
        // Therefore there is no point to preserve the original encoding.
        frame->img = cv_bridge::toCvShare(img_msg, "mono8")->image;
      } catch (cv_bridge::Exception &e) {
        RCLCPP_INFO(node->get_logger(), "cv_bridge exception: %s", e.what());
        continue;
      } catch (std::exception &e) {
        RCLCPP_INFO(node->get_logger(), "Exception: %s", e.what());
        continue;
      }
      *next_frame = std::move(frame);
      // Record current image message as already processed
      prev_img_msg = img_msg.get();
    }
  }
  EXPECT_END_OF_STREAM
};
