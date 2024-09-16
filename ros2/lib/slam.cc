// =============================================================================
// TODO: Add file description
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================

#include "common.h"
#include "types.h"
#include <memory>
#include <sstream>
#include <string>

// SLAM_Node Methods
SLAM::SLAM(SensorType sensor_type)
    : Node("ORB_SLAM3"), sensor_type(sensor_type),
      use_imu(sensor_type == ORB_SLAM3::System::IMU_MONOCULAR ||
              sensor_type == ORB_SLAM3::System::IMU_STEREO ||
              sensor_type == ORB_SLAM3::System::IMU_RGBD),
      use_stereo(sensor_type == ORB_SLAM3::System::STEREO ||
                 sensor_type == ORB_SLAM3::System::IMU_STEREO) {
  RCLCPP_INFO(get_logger(), "Launching from: %s", getcwd(nullptr, 0));
  init_parameters();
  init_topics();
  init_services();
  // Start sync thread only if IMU is enabled
  if (use_imu)
    sync = std::make_unique<Sync>(this, &next_frame);
  // Start main SLAM loop
  thread = std::make_unique<std::thread>(&SLAM::loop, this);
}

SLAM::~SLAM() {
  // sync->terminate();
  RCLCPP_INFO(get_logger(), "Stopping SLAM::thread");
  flag_term = true;
  if (thread && thread->joinable())
    thread->join();
  else
    RCLCPP_INFO(get_logger(), "SLAM::thread not joinable");
  RCLCPP_INFO(get_logger(), "Stopping Sync::thread");
  if (sync)
    sync->terminate();
}

void normalize_path(std::string &path) {
  // Change to absolute path
  if (path[0] != '/')
    path = std::string(getcwd(nullptr, 0)) + "/" + path;
  // Check if file exists
  if (!std::ifstream(path).good()) {
    throw std::runtime_error("File not found at: " + path);
  }
}

void SLAM::init_parameters() {
  declare_parameter<bool>("enable_pangolin", false);
  get_parameter("enable_pangolin", param.enable_pangolin);
  declare_parameter("voc_file", "");
  if (!get_parameter("voc_file", param.input_file.vocabulary)) {
    RCLCPP_ERROR(get_logger(), "parameter <voc_file> not set");
    throw std::runtime_error("voc_file not set");
  };
  normalize_path(param.input_file.vocabulary);
  declare_parameter("settings_file", "");
  if (!get_parameter("settings_file", param.input_file.settings)) {
    RCLCPP_ERROR(get_logger(), "parameter <settings_file> not set");
    throw std::runtime_error("settings_file not set");
  };
  normalize_path(param.input_file.settings);
  declare_parameter("world_frame_id", "map");
  get_parameter("world_frame_id", param.frame_id.world);
  declare_parameter("cam_frame_id", "camera");
  get_parameter("cam_frame_id", param.frame_id.cam);
  declare_parameter("imu_frame_id", "imu");
  get_parameter("imu_frame_id", param.frame_id.imu);
}

void SLAM::init_topics() {
  const auto imu_msg_handler = [this](msg::IMU::SharedPtr msg) {
    if (this->sync == nullptr)
      return;
    this->sync->handle_imu_msg(msg);
  };
  sub.imu = create_subscription<msg::IMU>("imu/get", 10, imu_msg_handler);

  const auto img_msg_handler = [this](msg::Image::SharedPtr msg) {
    if (this->sync == nullptr)
      return;
    this->sync->handle_img_msg(msg);
  };
  sub.img = create_subscription<msg::Image>("img/get", 10, img_msg_handler);

  pub.pose = create_publisher<msg::Pose>("pose", 10);
  pub.transform = create_publisher<msg::Transform>("transform", 10);
  pub.kf_markers = create_publisher<msg::VisMarker>("kf_markers", 10);
  pub.all_points = create_publisher<msg::PointCloud>("all_points", 10);
  pub.tracked_points = create_publisher<msg::PointCloud>("tracked_points", 10);
  pub.tracking_image = create_publisher<msg::Image>("tracking_image", 10);
  // IMU-specific topics
  if (use_imu) {
    pub.imu.odom = create_publisher<msg::Odometry>("imu/odom", 10);
    pub.imu.transform = create_publisher<msg::Transform>("imu/transform", 10);
  }
}

void SLAM::init_services() {
  srv.save_map = create_service<orb_slam3_interfaces::srv::Save>(
      "slam/map/save",
      [this](const orb_slam3_interfaces::srv::Save::Request::SharedPtr req,
             orb_slam3_interfaces::srv::Save::Response::SharedPtr res) {
        RCLCPP_INFO(get_logger(), "Saving Map [NOT IMPLEMENTED]");
        res->success = system->SaveMap(req->name);
      });
  srv.save_trj = create_service<orb_slam3_interfaces::srv::Save>(
      "slam/trj/save",
      [this](const orb_slam3_interfaces::srv::Save::Request::SharedPtr req,
             orb_slam3_interfaces::srv::Save::Response::SharedPtr res) {
        RCLCPP_INFO(get_logger(), "Saving Trajectory");
        try {
          system->SaveKeyFrameTrajectoryEuRoC(req->name);
          res->success = true;
        } catch (std::exception &e) {
          RCLCPP_ERROR(get_logger(), "Failed to save trajectory: %s", e.what());
          res->success = false;
        }
      });
}

void SLAM::loop() {
  try { // Initialize ORB_SLAM3
    system = std::make_unique<ORB_SLAM3::System>(
        param.input_file.vocabulary, param.input_file.settings,
        ORB_SLAM3::System::MONOCULAR, param.enable_pangolin);
    std::shared_ptr<const DataFrame> frame = nullptr;
    try {
      RCLCPP_INFO(get_logger(), "[SLAM::loop] Starting Loop ...");
      while (!flag_term) {
        std::stringstream ss;
        // Consume the latest frame
        next_frame.next(frame, true);
        // Process next frame
        auto &img = frame->img;
        auto seconds = frame->time.seconds();
        auto &imu = frame->imu;
        ss << "Frame IMU(" << imu.size() << ")" << " ";
        system->TrackMonocular(img, seconds, imu);
        ss << "State(" << system->GetTrackingStateName() << ")";
        RCLCPP_INFO(get_logger(), "[SLAM::loop] %s", ss.str().c_str());
        publish(frame->time, frame->Wbb);
      }
    }
    EXPECT_END_OF_STREAM
    next_frame.close();
    // Save map and trajectory
    system->SaveMap("map");
    system->SaveTrajectoryEuRoC("trj");
  }
  EXPECT_END_OF_STREAM
  catch (std::exception &e) {
    RCLCPP_ERROR(get_logger(), "[SLAM::loop] %s", e.what());
    return;
  }
  catch (...) {
    RCLCPP_ERROR(get_logger(), "[SLAM::loop] Unknown Error");
    return;
  }
}

void SLAM::publish(const Time &time, const Eigen::Vector3f &Wbb) {
  msg::Header header;
  header.stamp = time;
  header.frame_id = param.frame_id.world;
  Sophus::SE3f Twc = system->GetCamTwc();
  if (Twc.translation().array().isNaN().any() ||
      Twc.rotationMatrix().array().isNaN().any())
    return;
  pub.pose->publish(*msg::camera_pose(header, Twc));
  pub.transform->publish(*msg::tf_transform(header, Twc));
  pub.kf_markers->publish(
      *msg::kf_markers(header, system->GetAllKeyframePoses()));
  pub.all_points->publish(
      *msg::from_map_point(header, system->GetAllMapPoints()));

  std::vector<ORB_SLAM3::MapPoint *> tracked_points;
  for (auto const &point : system->GetTrackedMapPoints()) {
    if (point)
      tracked_points.push_back(point);
  }
  if (!tracked_points.empty())
    pub.tracked_points->publish(*msg::from_map_point(header, tracked_points));

  pub.tracking_image->publish(
      *msg::tracking_img(header, system->GetCurrentFrame()));

  // IMU-specific topics
  if (use_imu) {
    // Body pose and translational velocity can be obtained from ORB-SLAM3
    Sophus::SE3f Twb = system->GetImuTwb();
    Eigen::Vector3f Vwb = system->GetImuVwb();

    // IMU provides body angular velocity in body frame (Wbb) which is
    // transformed to world frame (Wwb)
    Sophus::Matrix3f Rwb = Twb.rotationMatrix();
    Eigen::Vector3f Wwb = Rwb * Wbb;
    pub.imu.transform->publish(*msg::tf_transform(header, Twb));
    pub.imu.odom->publish(
        *msg::body_odom(header, param.frame_id.imu, Twb, Vwb, Wwb));
  }
};