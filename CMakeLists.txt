cmake_minimum_required(VERSION 3.8)
project(kromek_ros2_hidapi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Find HIDAPI
find_package(PkgConfig REQUIRED)
pkg_check_modules(HIDAPI REQUIRED hidapi-libusb)

# Include directories
include_directories(
  include
  ${HIDAPI_INCLUDE_DIRS}
)

# Create the ROS2 node executable
add_executable(kromek_hidapi_node src/kromek_hidapi_node.cpp)

# Link dependencies
ament_target_dependencies(kromek_hidapi_node
  PUBLIC
  rclcpp
  std_msgs
)

# Link HIDAPI
target_link_libraries(kromek_hidapi_node
  PUBLIC
  ${HIDAPI_LIBRARIES}
)

# Install the executable
install(TARGETS
  kromek_hidapi_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
