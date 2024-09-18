// =============================================================================
// Common message types used by the SLAM system.
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================

#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "threading/leaky.h"
#include "threading/fifo.h"
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <ORB_SLAM3/ORB_SLAM3.h>

using Time = rclcpp::Time;

namespace msg {
  
using Header = std_msgs::msg::Header;

using Odometry = nav_msgs::msg::Odometry;

using IMU = sensor_msgs::msg::Imu;
using Image = sensor_msgs::msg::Image;
using PointField = sensor_msgs::msg::PointField;
using PointCloud = sensor_msgs::msg::PointCloud2;

using Pose = geometry_msgs::msg::PoseStamped;
using Point = geometry_msgs::msg::Point;
using Transform = geometry_msgs::msg::TransformStamped;

using VisMarker = visualization_msgs::msg::Marker;

} // namespace msg

typedef struct DataFrame {
  Time time;
  cv::Mat img;
  std::vector<ORB_SLAM3::IMU::Point> imu;
  Eigen::Vector3f Wbb;
} DataFrame;

typedef threading::LeakyIO<DataFrame> DataFramePipe;
typedef threading::FIFO<msg::Image::SharedPtr> ImagePipe;
typedef threading::FIFO<msg::IMU> IMUPipe;
