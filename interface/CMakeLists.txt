cmake_minimum_required(VERSION 3.8)
project(orb_slam3_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# Generate ROS2 Serivce Interface
find_package(rosidl_default_generators REQUIRED)
# Find all .srv and .msg files inside this package
FILE(GLOB INTERFACES *.srv **/*.srv *.msg **/*.msg)
# Convert to relative path
STRING(REPLACE "${CMAKE_CURRENT_SOURCE_DIR}/" "" INTERFACES "${INTERFACES}")
# Generate ROS2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME} ${INTERFACES})

ament_package()
