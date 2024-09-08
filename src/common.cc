#include "common.h"

// SLAM_Node Methods
SLAM::SLAM(SensorType sensor_type, const char *const pixel_format)
    : Node("ORB_SLAM3"),
      enable_imu(sensor_type == ORB_SLAM3::System::IMU_MONOCULAR ||
                 sensor_type == ORB_SLAM3::System::IMU_STEREO ||
                 sensor_type == ORB_SLAM3::System::IMU_RGBD) {
  init_parameters();
  init_topics();
  init_services();
  // Initialize ORB_SLAM3
  system = std::make_unique<ORB_SLAM3::System>(voc_file, settings_file,
                                               sensor_type, true);
  // Start sync thread only if IMU is enabled
  if (enable_imu)
    sync = std::make_unique<Sync>(this, pixel_format);
}

SLAM::~SLAM() {
  RCLCPP_INFO(get_logger(), "Stopping Sync Thread");
  sync = nullptr;
  RCLCPP_INFO(get_logger(), "Stopping ORB_SLAM3 System");
  system = nullptr;
}

void SLAM::init_parameters() {
  declare_parameter<bool>("enable_pangolin", false);
  get_parameter("enable_pangolin", enable_pangolin);
  declare_parameter("voc_file", "");
  if (!get_parameter("voc_file", voc_file)) {
    RCLCPP_ERROR(get_logger(), "parameter <voc_file> not set");
    throw std::runtime_error("voc_file not set");
  };
  declare_parameter("settings_file", "");
  if (!get_parameter("settings_file", settings_file)) {
    RCLCPP_ERROR(get_logger(), "parameter <settings_file> not set");
    throw std::runtime_error("settings_file not set");
  };
  declare_parameter("world_frame_id", "map");
  get_parameter("world_frame_id", world_frame_id);
  declare_parameter("cam_frame_id", "camera");
  get_parameter("cam_frame_id", cam_frame_id);
  declare_parameter("imu_frame_id", "imu");
  get_parameter("imu_frame_id", imu_frame_id);
}

void SLAM::init_topics() {
  imu_sub = create_subscription<msg::IMU>(
      "imu/get", 10, [this](const msg::IMU::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Received IMU ACC = [%f, %f, %f]",
                    msg->linear_acceleration.x, msg->linear_acceleration.y,
                    msg->linear_acceleration.z);
      });
  img_sub = create_subscription<msg::Image>(
      "camera/get", 10, [this](const msg::Image::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Received Image");
      });
  pose_pub = create_publisher<msg::PoseStamped>("pose", 10);
  odom_pub = create_publisher<msg::Odometry>("odom", 10);
  kf_markers_pub = create_publisher<msg::VisMarker>("kf_markers", 10);
  all_points_pub = create_publisher<msg::PointCloud>("all_points", 10);
  tracked_points_pub = create_publisher<msg::PointCloud>("tracked_points", 10);
  tracked_key_points_pub =
      create_publisher<msg::PointCloud>("tracked_key_points", 10);
  tracking_image_pub = create_publisher<msg::Image>("tracking_image", 10);
}

void SLAM::init_services() {
  save_map_srv = create_service<orb_slam3::srv::SaveMap>(
      "slam/map/save",
      [this](const orb_slam3::srv::SaveMap::Request::SharedPtr req,
             orb_slam3::srv::SaveMap::Response::SharedPtr res) {
        RCLCPP_INFO(get_logger(), "Saving Map [NOT IMPLEMENTED]");
        res->success = system->SaveMap(req->name);
      });
  save_trj_srv = create_service<orb_slam3::srv::SaveMap>(
      "slam/trj/save",
      [this](const orb_slam3::srv::SaveMap::Request::SharedPtr req,
             orb_slam3::srv::SaveMap::Response::SharedPtr res) {
        RCLCPP_INFO(get_logger(), "Saving Trajectory");
        system->SaveKeyFrameTrajectoryEuRoC(req->name);
        res->success = true;
      });
}