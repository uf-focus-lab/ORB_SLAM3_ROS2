#include "sync.h"
#include "types.h"

#include "orb_slam3_interfaces/srv/save.hpp"

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

typedef struct NodeParam_s {
  bool enable_pangolin;
  struct {
    std::string vocabulary;
    std::string settings;
  } input_file;
  struct {
    std::string world;
    std::string cam;
    std::string imu;
  } frame_id;
} NodeParam;

class SLAM : public rclcpp::Node {
  std::unique_ptr<DataFrame> next_frame;

public:
  // ORB_SLAM3 System Instance;
  std::unique_ptr<ORB_SLAM3::System> system;
  std::unique_ptr<Sync> sync;

  // Startup parameters
  const SensorType sensor_type;
  const bool use_imu, use_stereo;

  // Configurable entries
  NodeParam param;

  // Subscribers
  struct {
    rclcpp::Subscription<msg::IMU>::SharedPtr imu;
    rclcpp::Subscription<msg::Image>::SharedPtr img;
  } sub;

  // Publishers
  struct {
    rclcpp::Publisher<msg::Pose>::SharedPtr pose;
    rclcpp::Publisher<msg::Transform>::SharedPtr transform;
    rclcpp::Publisher<msg::VisMarker>::SharedPtr kf_markers;
    rclcpp::Publisher<msg::PointCloud>::SharedPtr all_points;
    rclcpp::Publisher<msg::PointCloud>::SharedPtr tracked_points;
    rclcpp::Publisher<msg::Image>::SharedPtr tracking_image;
    // IMU-specific topics
    struct {
      rclcpp::Publisher<msg::Odometry>::SharedPtr odom;
      rclcpp::Publisher<msg::Transform>::SharedPtr transform;
    } imu;
  } pub;

  // Topic publisher utility
  void publish(Time &time, Eigen::Vector3f &Wbb);

  // Services
  struct {
    rclcpp::Service<orb_slam3_interfaces::srv::Save>::SharedPtr save_map;
    rclcpp::Service<orb_slam3_interfaces::srv::Save>::SharedPtr save_trj;
  } srv;

  void init_parameters();
  void init_topics();
  void init_services();

  std::unique_ptr<std::thread> thread;
  bool flag_term = false;

  void loop();

public:
  SLAM(SensorType sensor_type);
  ~SLAM();
};
