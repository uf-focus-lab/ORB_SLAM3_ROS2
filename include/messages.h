// =============================================================================
// Common message types used by the SLAM system.
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

using Time = rclcpp::Time;

namespace msg {
using IMU = sensor_msgs::msg::Imu;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Odometry = nav_msgs::msg::Odometry;
using VisMarker = visualization_msgs::msg::Marker;
using PointCloud = sensor_msgs::msg::PointCloud2;
using Image = sensor_msgs::msg::Image;
} // namespace msg
