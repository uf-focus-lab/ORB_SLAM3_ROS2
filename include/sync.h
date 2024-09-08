// =============================================================================
// Sensor message synchronization
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================

#pragma once

#include "messages.h"
#include "slam.h"

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.hpp>

class Sync {
private:
  SLAM *node;
  const char *const pixel_format;
  std::mutex imu_lock, img_lock;
  std::queue<msg::IMU::SharedPtr> imu_queue;
  std::queue<msg::Image::SharedPtr> img_queue;
  // Standalone thread for synchronization
  std::unique_ptr<std::thread> thread;
  bool flag_term = false;

public:
  Sync(rclcpp::Node *node, const char *const pixel_format);
  ~Sync() {
    flag_term = true;
    thread->join();
  }
  void GrabImage(const msg::Image::SharedPtr img_msg) {
    img_lock.lock();
    if (!img_queue.empty())
      img_queue.pop();
    img_queue.push(img_msg);
    img_lock.unlock();
  };
  cv::Mat GetImage(const msg::Image::SharedPtr img_msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(img_msg, pixel_format);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_INFO(node->get_logger(), "cv_bridge exception: %s", e.what());
    }
    if (cv_ptr->image.type() == 0) {
      return cv_ptr->image.clone();
    } else {
      std::cout << "Error type" << std::endl;
      return cv_ptr->image.clone();
    }
  }

  void GrabImu(const msg::IMU::SharedPtr imu_msg) {

  };

  void loop() {
    while (!flag_term) {
      // Synchronize IMU and Image
      imu_lock.lock();
      img_lock.lock();
      if (!imu_queue.empty() && !img_queue.empty()) {
        auto imu_msg = imu_queue.front();
        auto img_msg = img_queue.front();
        // Process imu_msg and img_msg
        RCLCPP_INFO(node->get_logger(), "Synced IMU and Image");
        // Pop the synchronized messages
        imu_queue.pop();
        img_queue.pop();
      }
      imu_lock.unlock();
      img_lock.unlock();
    }
  };
};
