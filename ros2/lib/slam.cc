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
    : Node("ORB_SLAM3"), sensor_type(sensor_type) {
  RCLCPP_INFO(get_logger(), "Launching from: %s", getcwd(nullptr, 0));
  param.use_stereo =
      (sensor_type & SensorType::CAMERA_MASK) == SensorType::STEREO;
  init_parameters();
  init_topics();
  init_services();
  // Update Sensor Type based on user supplied parameters
  if (param.use_imu)
    sensor_type = static_cast<SensorType>(sensor_type | SensorType::USE_IMU);
  // Start sync thread only if IMU is enabled
  sync = std::make_unique<Sync>(this, &next_frame);
  // Start main SLAM loop
  thread = std::make_unique<std::thread>(&SLAM::loop, this);
}

SLAM::~SLAM() {
  flag_term = true;
  next_frame.close();
  // Stop sync thread if enabled
  std::cerr << "Stopping sync thread" << std::endl;
  sync = nullptr;
  RCLCPP_INFO(get_logger(), "Waiting for SLAM::thread");
  if (thread)
    thread->join();
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
  declare_parameter<bool>("use_imu", false);
  get_parameter("use_imu", param.use_imu);
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
    if (this->sync)
      this->sync->handle_imu_msg(msg);
  };
  sub.imu = create_subscription<msg::IMU>("imu/get", 10, imu_msg_handler);

  const auto img_msg_handler = [this](msg::Image::SharedPtr msg) {
    if (this->sync)
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
  if (param.use_imu) {
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
        param.input_file.vocabulary, param.input_file.settings, sensor_type,
        param.enable_pangolin);
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
  // catch (std::exception &e) {
  //   RCLCPP_ERROR(get_logger(), "[SLAM::loop] Exception: %s", e.what());
  //   return;
  // }
  // catch (...) {
  //   RCLCPP_ERROR(get_logger(), "[SLAM::loop] Unknown Error");
  //   return;
  // }
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

  auto T_bc = system->GetSettings().Tbc();

  std::vector<Eigen::Vector3f> all_map_points;
  for (auto const &point : system->GetAllMapPoints()) {
    if (!point)
      continue;
    Eigen::Vector4f p;
    p << point->GetWorldPos(), 1;
    p = T_bc * p;
    all_map_points.push_back(p.head<3>());
  }
  if (!all_map_points.empty())
    pub.all_points->publish(*msg::from_map_point(header, all_map_points));

  std::vector<Eigen::Vector3f> tracked_points;
  for (auto const &point : system->GetTrackedMapPoints()) {
    if (!point)
      continue;
    Eigen::Vector4f p;
    p << point->GetWorldPos(), 1;
    p = T_bc * p;
    tracked_points.push_back(p.head<3>());
  }
  if (!tracked_points.empty())
    pub.tracked_points->publish(*msg::from_map_point(header, tracked_points));

  pub.tracking_image->publish(
      *msg::tracking_img(header, system->GetCurrentFrame()));

  // IMU-specific topics
  if (param.use_imu) {
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