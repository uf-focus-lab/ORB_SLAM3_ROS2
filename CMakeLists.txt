cmake_minimum_required(VERSION 3.8)
project(ORB_SLAM3_ROS2)

# Set the build type. Options are:
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/ORB_SLAM3/scripts)
include(compiler)

find_package(OpenCV 4.2)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV 4.2 not found.")
endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  tf
  tf2
  std_msgs
  nav_msgs
  cv_bridge
  sensor_msgs
  geometry_msgs
  image_transport
  message_filters
  message_generation
)

# Driver Dependencies
find_package(OpenCV REQUIRED)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# ROS2 packages
find_package(tf REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(message_generation REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package (
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf sensor_msgs dynamic_reconfigure message_runtime
  LIBRARIES {PROJECT_NAME} libDBoW2 libg2o
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/ORB_SLAM3/include
  ${OpenCV_INCLUDE_DIRS}
)

link_libraries(
  ${OpenCV_LIBS}
  ${PROJECT_SOURCE_DIR}/ORB_SLAM3/lib/libORB_SLAM3.so
  -lboost_system
  -lboost_serialization
  -lcrypto
)

set(NODES
  ros_mono
  ros_mono_inertial
  ros_stereo
  ros_stereo_inertial
  ros_rgbd
  ros_rgbd_inertial
)

foreach(NODE ${NODES})
  add_executable(${NODE} src/${NODE}.cc src/common.cc)
  ament_target_dependencies(${NODE}
    tf
    tf2
    std_msgs
    nav_msgs
    cv_bridge
    sensor_msgs
    geometry_msgs
    image_transport
    message_filters
    message_generation
  )
endforeach()

install(TARGETS ${NODES}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()