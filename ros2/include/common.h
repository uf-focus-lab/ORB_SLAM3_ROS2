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

#include "slam.h"      // IWYU pragma: export
#include "threading.h" // IWYU pragma: export
#include "types.h"     // IWYU pragma: export

// ROS2 Node
extern std::shared_ptr<SLAM> slam;

// Message Handlers
void handle_imu_message(const msg::IMU::SharedPtr msg);
void handle_image_message(const msg::Image::SharedPtr msg);

// Sophus to ROS2 Message Converters
namespace msg {
Pose::SharedPtr camera_pose(const Header &, Sophus::SE3f &T_cw);
VisMarker::SharedPtr kf_markers(const Header &,
                                const std::vector<Sophus::SE3f> &kf_poses);
Transform::SharedPtr tf_transform(const Header &, const Sophus::SE3f &T_SE3f);
Image::SharedPtr tracking_img(const Header &, const cv::Mat &image);
Odometry::SharedPtr body_odom(const Header &, const std::string &imu_frame_id,
                              const Sophus::SE3f &Twb_SE3f,
                              const Eigen::Vector3f &Vwb_E3f,
                              const Eigen::Vector3f &ang_vel_body);
PointCloud::SharedPtr
from_map_point(const Header &,
               const std::vector<ORB_SLAM3::MapPoint *> map_points);
} // namespace msg

// Utility Functions
cv::Mat SE3f_to_cvMat(const Sophus::SE3f &T);
