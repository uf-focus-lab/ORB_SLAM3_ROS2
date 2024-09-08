#include "sync.h"

Sync::Sync(SLAM *node, const char *const pixel_format)
    : node(node), pixel_format(pixel_format) {
  thread = std::make_unique<std::thread>(&Sync::loop, this);
}

void Sync::GrabImage(const msg::Image::SharedPtr img_msg) {
  img_lock.lock();
  if (!img_queue.empty())
    img_queue.pop();
  img_queue.push(img_msg);
  img_lock.unlock();
}

cv::Mat Sync::GetImage(const msg::Image::SharedPtr img_msg) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(img_msg, T);
  } catch (cv_bridge::Exception &e) {
    ("cv_bridge exception: %s", e.what());
  }

  if (cv_ptr->image.type() == 0) {
    return cv_ptr->image.clone();
  } else {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void Sync::sync() {
}

void Sync::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg) {
  img_lock.lock();
  imuBuf.push(imu_msg);
  img_lock.unlock();

  return;
}
