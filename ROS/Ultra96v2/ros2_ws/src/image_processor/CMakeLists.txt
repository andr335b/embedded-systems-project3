cmake_minimum_required(VERSION 3.5)
project(image_processor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Add executable
add_executable(image_processor_node src/image_processor_node.cpp)
ament_target_dependencies(image_processor_node rclcpp sensor_msgs cv_bridge)

# Install
install(TARGETS
  image_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
