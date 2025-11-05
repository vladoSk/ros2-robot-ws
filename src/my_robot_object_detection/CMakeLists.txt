cmake_minimum_required(VERSION 3.8)
project(my_robot_object_detection)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# Object Detector Node
add_executable(object_detector src/object_detector.cpp)
ament_target_dependencies(object_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
  OpenCV
)

# Object Follower Node
add_executable(object_follower src/object_follower.cpp)
ament_target_dependencies(object_follower
  rclcpp
  geometry_msgs
  std_msgs
)

# Install executables
install(TARGETS
  object_detector
  object_follower
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
