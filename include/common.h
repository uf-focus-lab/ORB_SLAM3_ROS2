// =============================================================================
// Common includes and abstract classes.
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================

#pragma once

#include <algorithm> // IWYU pragma: export
#include <chrono>    // IWYU pragma: export
#include <cmath>     // IWYU pragma: export
#include <fstream>   // IWYU pragma: export
#include <iostream>  // IWYU pragma: export
#include <memory>    // IWYU pragma: export
#include <mutex>     // IWYU pragma: export
#include <queue>     // IWYU pragma: export
#include <string>    // IWYU pragma: export
#include <thread>    // IWYU pragma: export
#include <vector>    // IWYU pragma: export

#include <rclcpp/rclcpp.hpp>               // IWYU pragma: export
#include <sensor_msgs/image_encodings.hpp> // IWYU pragma: export

#include "slam.h"

// ROS2 Node
extern std::shared_ptr<SLAM> slam;
// Subscribers
extern rclcpp::Subscription<msg::IMU>::SharedPtr imu_sub;
extern rclcpp::Subscription<msg::Image>::SharedPtr img_sub;
// Publishers
extern rclcpp::Publisher<msg::PoseStamped>::SharedPtr pose_pub;
extern rclcpp::Publisher<msg::Odometry>::SharedPtr odom_pub;
extern rclcpp::Publisher<msg::VisMarker>::SharedPtr kf_markers_pub;
extern rclcpp::Publisher<msg::PointCloud>::SharedPtr all_points_pub;
extern rclcpp::Publisher<msg::PointCloud>::SharedPtr tracked_points_pub;
extern rclcpp::Publisher<msg::PointCloud>::SharedPtr tracked_key_points_pub;
extern rclcpp::Publisher<msg::Image>::SharedPtr tracking_image_pub;
// Message Handlers
void handle_imu_message(const msg::IMU::SharedPtr msg);
void handle_image_message(const msg::Image::SharedPtr msg);
// Message Publishers
void publish_camera_pose(const Time &time);
void publish_kf_markers(const Time &time);
void publish_tf_transform(const Time &time);
void publish_all_points(const Time &time);
void publish_tracked_points(const Time &time);
void publish_tracked_key_points(const Time &time);
void publish_tracking_img(const Time &time);
// Utility Functions
cv::Mat SE3f_to_cvMat(const Sophus::SE3f &T);
