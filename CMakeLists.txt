cmake_minimum_required(VERSION 3.8)
project(orb_slam3)

# Install directory for ORB_SLAM3 core libraries
set(CORE_INSTALL_DIR ${PROJECT_SOURCE_DIR}/core/install)
# Set the build type. Options are:
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/core/scripts)
include(compiler)

# Ask OpenGL to use GLVND when available (instead of LEGACY).
# Run "cmake --help-policy CMP0072" for policy details.
cmake_policy(SET CMP0072 NEW)

set(ROS2DEPS
  # Transforms Library
  tf2
  # ROS2 Messages
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  # CV / Image Processing Libraries
  cv_bridge
  image_transport
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
# ROS2 packages
foreach(DEP IN ITEMS ${ROS2DEPS})
  find_package(${DEP} REQUIRED)
endforeach()
# ORB_SLAM3 libraires
find_package(DBoW2 REQUIRED PATHS ${CORE_INSTALL_DIR})
find_package(g2o REQUIRED PATHS ${CORE_INSTALL_DIR})
find_package(Pangolin REQUIRED PATHS ${CORE_INSTALL_DIR})
find_package(Sophus REQUIRED PATHS ${CORE_INSTALL_DIR})
find_package(ORB_SLAM3 REQUIRED PATHS ${CORE_INSTALL_DIR})
# Generate ROS2 Serivce Interface
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "srv/SaveMap.srv")
find_package(${PROJECT_NAME} REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIR}
  ${DBoW2_INCLUDE_DIR}
  ${ORB_SLAM3_INCLUDE_DIR}
)

link_libraries(
  -lcrypto
  -lboost_system
  -lboost_serialization
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${DBoW2_LIBRARIES}
  ${G2O_LIBS}
  ${ORB_SLAM3_LIBS}
)

set(NODES
  mono_inertial
  # ros_mono
  # ros_mono_inertial
  # ros_stereo
  # ros_stereo_inertial
  # ros_rgbd
  # ros_rgbd_inertial
)

foreach(NODE ${NODES})
  add_executable(${NODE}
    src/${NODE}.cc
    # src/common.cc
  )
  ament_target_dependencies(${NODE} ${ROS2DEPS} ${PROJECT_NAME})
  rosidl_target_interfaces(${NODE} ${PROJECT_NAME})
endforeach()

install(TARGETS ${NODES} DESTINATION lib/${PROJECT_NAME})
# Install submodule libraries
# NOTE: You need to first run `make` in the ORB_SLAM3 directory
install(DIRECTORY ${CORE_INSTALL_DIR} DESTINATION ${CMAKE_INSTALL_PREFIX})

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
