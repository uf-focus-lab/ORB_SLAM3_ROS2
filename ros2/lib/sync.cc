// =============================================================================
// TODO: Add file description
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================

#include "sync.h"
#include "types.h"
#include <memory>
#include <rclcpp/logging.hpp>

using namespace std::chrono_literals;

Sync::Sync(rclcpp::Node *node, DataFramePipe *frame_out)
    : node(node), frame_out(frame_out) {
  thread = std::make_unique<std::thread>(&Sync::loop, this);
}

void Sync::terminate() {
  RCLCPP_INFO(node->get_logger(), "Terminating sync thread");
  flag_term = true;
  img_in.close();
  imu_in.close();
  frame_out->close();
  thread->join();
  RCLCPP_INFO(node->get_logger(), "Sync thread terminated");
}

void Sync::handle_img_msg(msg::Image::SharedPtr img_msg) {
  img_in.write(*img_msg);
};

void Sync::handle_imu_msg(msg::IMU::SharedPtr imu_msg) {
  imu_in.write(*imu_msg);
};

ORB_SLAM3::IMU::Point imu_point(msg::IMU imu_msg) {
  rclcpp::Time time(imu_msg.header.stamp);
  const auto &acc = imu_msg.linear_acceleration;
  const auto &ang = imu_msg.angular_velocity;
  return {static_cast<float>(acc.x),
          static_cast<float>(acc.y),
          static_cast<float>(acc.z),
          static_cast<float>(ang.x),
          static_cast<float>(ang.y),
          static_cast<float>(ang.z),
          time.seconds()};
}

void Sync::loop() {
  try {
    // Load first IMU message
    std::shared_ptr<const msg::Image> img_msg;
    while (!flag_term) {
      // Only look once
      img_in.next(img_msg, true);
      rclcpp::Time img_time(img_msg->header.stamp);
      auto frame = DataFrame();
      frame.time = img_time;
      // Stack all IMU messages before current image taken
      while (!imu_in.empty()) {
        rclcpp::Time imu_time(imu_in.peek().header.stamp);
        if (imu_time > img_time)
          break;
        const auto point = imu_point(imu_in.read());
        frame.imu.push_back(point);
        frame.Wbb << point.w.x(), point.w.y(), point.w.z();
      }
      // Synchronize IMU and Image
      try {
        // ORB_SLAM3 will convert all images to grayscale (U8).
        // Therefore there is no point to preserve the original encoding.
        frame.img = cv_bridge::toCvShare(img_msg, "mono8")->image;
      } catch (cv_bridge::Exception &e) {
        RCLCPP_INFO(node->get_logger(), "cv_bridge exception: %s", e.what());
        continue;
      } catch (std::exception &e) {
        RCLCPP_INFO(node->get_logger(), "Exception: %s", e.what());
        continue;
      }
      frame_out->write(frame);
    }
  }
  EXPECT_END_OF_STREAM
  img_in.close();
  imu_in.close();
  frame_out->close();
};
