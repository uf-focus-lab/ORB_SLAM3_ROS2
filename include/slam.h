#include "messages.h"
#include "sync.h"

#include "orb_slam3/srv/save_map.hpp"

#include <memory>

#include <Eigen/Dense>
#include <ORB_SLAM3/ORB_SLAM3.h>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sophus/se3.hpp>

typedef const enum ORB_SLAM3::System::eSensor SensorType;

class SLAM : public rclcpp::Node {
public:
  // ORB_SLAM3 System Instance;
  std::unique_ptr<ORB_SLAM3::System> system;
  std::unique_ptr<Sync> sync;

  // Configurable entries
  bool enable_pangolin;
  bool enable_imu;
  std::string voc_file;
  std::string settings_file;
  std::string world_frame_id;
  std::string cam_frame_id;
  std::string imu_frame_id;

  // Subscribers
  rclcpp::Subscription<msg::IMU>::SharedPtr imu_sub;
  rclcpp::Subscription<msg::Image>::SharedPtr img_sub;

  // Publishers
  rclcpp::Publisher<msg::PoseStamped>::SharedPtr pose_pub;
  rclcpp::Publisher<msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<msg::VisMarker>::SharedPtr kf_markers_pub;
  rclcpp::Publisher<msg::PointCloud>::SharedPtr all_points_pub;
  rclcpp::Publisher<msg::PointCloud>::SharedPtr tracked_points_pub;
  rclcpp::Publisher<msg::PointCloud>::SharedPtr tracked_key_points_pub;
  rclcpp::Publisher<msg::Image>::SharedPtr tracking_image_pub;

  // Services
  rclcpp::Service<orb_slam3::srv::SaveMap>::SharedPtr save_map_srv;
  rclcpp::Service<orb_slam3::srv::SaveMap>::SharedPtr save_trj_srv;

  // Publish Utility Functions
  void publish_camera_pose(Time &time);
  void publish_kf_markers(Time &time);
  void publish_tf_transform(Time &time);
  void publish_all_points(Time &time);
  void publish_tracked_points(Time &time);
  void publish_tracked_key_points(Time &time);
  void publish_tracking_img(Time &time);

  void init_parameters();
  void init_topics();
  void init_services();

public:
  SLAM(SensorType sensor_type, const char *pixel_format);
  ~SLAM();
};
