cmake_minimum_required(VERSION 3.8)
project(probabilistic_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(kalmanFilter_node src/kalmanFilter_node.cpp)
ament_target_dependencies(kalmanFilter_node rclcpp geometry_msgs nav_msgs)

add_executable(extendedKalmanFilter_node src/extendedKalmanFilter_node.cpp)
ament_target_dependencies(extendedKalmanFilter_node rclcpp geometry_msgs nav_msgs)

add_executable(particleFilter_node src/particleFilter_node.cpp)
ament_target_dependencies(particleFilter_node rclcpp geometry_msgs nav_msgs)

add_executable(trajectory_node src/trajectory_node.cpp)
ament_target_dependencies(trajectory_node rclcpp geometry_msgs)

install(TARGETS
  kalmanFilter_node
  extendedKalmanFilter_node
  particleFilter_node
  trajectory_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
