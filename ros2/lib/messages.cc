// =============================================================================
// TODO: Add file description
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================

#include "common.h"
#include "types.h"

msg::Pose::SharedPtr msg::camera_pose(const msg::Header &header,
                                      Sophus::SE3f &Twc) {
  const auto msg = std::make_shared<msg::Pose>();
  msg->header = header;
  msg->pose.position.x = Twc.translation().x();
  msg->pose.position.y = Twc.translation().y();
  msg->pose.position.z = Twc.translation().z();
  msg->pose.orientation.w = Twc.unit_quaternion().coeffs().w();
  msg->pose.orientation.x = Twc.unit_quaternion().coeffs().x();
  msg->pose.orientation.y = Twc.unit_quaternion().coeffs().y();
  msg->pose.orientation.z = Twc.unit_quaternion().coeffs().z();
  return msg;
};

msg::VisMarker::SharedPtr
msg::kf_markers(const msg::Header &header,
                const std::vector<Sophus::SE3f> &kf_poses) {
  const auto msg = std::make_shared<msg::VisMarker>();
  msg->header = header;
  msg->id = 0;
  msg->ns = "kf_markers";
  msg->type = msg::VisMarker::SPHERE_LIST;
  msg->action = msg::VisMarker::ADD;
  msg->pose.orientation.w = 1.0;
  msg->scale.x = 0.05;
  msg->scale.y = 0.05;
  msg->scale.z = 0.05;
  msg->color.g = 1.0;
  msg->color.a = 1.0;
  for (const auto &el : kf_poses) {
    msg::Point point;
    point.x = el.translation().x();
    point.y = el.translation().y();
    point.z = el.translation().z();
    msg->points.push_back(point);
  }
  return msg;
};

msg::Transform::SharedPtr msg::tf_transform(const msg::Header &header,
                                            const Sophus::SE3f &T_SE3f) {
  const auto msg = std::make_shared<msg::Transform>();
  msg->header = header;
  auto q_vec = T_SE3f.unit_quaternion().coeffs();
  auto t_vec = T_SE3f.translation();
  msg->transform.rotation.w = q_vec.w();
  msg->transform.rotation.x = q_vec.x();
  msg->transform.rotation.y = q_vec.y();
  msg->transform.rotation.z = q_vec.z();
  msg->transform.translation.x = t_vec.x();
  msg->transform.translation.y = t_vec.y();
  msg->transform.translation.z = t_vec.z();
  return msg;
};

msg::Image::SharedPtr msg::tracking_img(const msg::Header &header,
                                        const cv::Mat &image) {
  const auto msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  msg->header = header;
  return msg;
};

msg::Odometry::SharedPtr msg::body_odom(const msg::Header &header,
                                        const std::string &imu_frame_id,
                                        const Sophus::SE3f &Twb_SE3f,
                                        const Eigen::Vector3f &Vwb_E3f,
                                        const Eigen::Vector3f &ang_vel_body) {
  const auto msg = std::make_shared<msg::Odometry>();
  msg->header = header;
  msg->child_frame_id = imu_frame_id;
  msg->pose.pose.position.x = Twb_SE3f.translation().x();
  msg->pose.pose.position.y = Twb_SE3f.translation().y();
  msg->pose.pose.position.z = Twb_SE3f.translation().z();
  msg->pose.pose.orientation.w = Twb_SE3f.unit_quaternion().coeffs().w();
  msg->pose.pose.orientation.x = Twb_SE3f.unit_quaternion().coeffs().x();
  msg->pose.pose.orientation.y = Twb_SE3f.unit_quaternion().coeffs().y();
  msg->pose.pose.orientation.z = Twb_SE3f.unit_quaternion().coeffs().z();
  msg->twist.twist.linear.x = Vwb_E3f.x();
  msg->twist.twist.linear.y = Vwb_E3f.y();
  msg->twist.twist.linear.z = Vwb_E3f.z();
  msg->twist.twist.angular.x = ang_vel_body.x();
  msg->twist.twist.angular.y = ang_vel_body.y();
  msg->twist.twist.angular.z = ang_vel_body.z();
  return msg;
};

msg::PointCloud::SharedPtr
msg::from_map_point(const msg::Header &header,
                    const std::vector<ORB_SLAM3::MapPoint *> map_points) {

  static const std::vector<std::string> channels = {"x", "y", "z"};
  typedef float F32;
  typedef struct {
    F32 x, y, z;
  } Point;

  const auto msg = std::make_shared<msg::PointCloud>();
  msg->header = header;
  msg->height = 1;
  msg->is_dense = true;
  msg->is_bigendian = false;
  msg->width = map_points.size();
  msg->point_step = channels.size() * sizeof(F32);
  msg->row_step = msg->point_step * msg->width;
  msg->fields.resize(0);
  msg::PointField field;
  field.count = 1;
  field.datatype = msg::PointField::FLOAT32;
  field.offset = msg->fields.size() * sizeof(F32);
  for (auto &channel : channels) {
    field.name = channel;
    msg->fields.push_back(field);
  }
  msg->data.resize(msg->row_step * msg->height);
  Point *cloud_data_ptr = (Point *)(void *)(&msg->data[0]);
  for (const auto p : map_points) {
    if (!p)
      continue;
    Eigen::Vector3f P3Dw = p->GetWorldPos();
    cloud_data_ptr->x = P3Dw.x();
    cloud_data_ptr->y = P3Dw.y();
    cloud_data_ptr->z = P3Dw.z();
    cloud_data_ptr++;
  }
  msg->data.resize((unsigned char *)cloud_data_ptr - &msg->data[0]);
  return msg;
}
