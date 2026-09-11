cmake_minimum_required(VERSION 3.8)
project(probabilistic_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Kalman Filter
add_executable(kalmanFilter_node src/kalmanFilter_node.cpp)
ament_target_dependencies(kalmanFilter_node rclcpp geometry_msgs nav_msgs)
target_link_libraries(kalmanFilter_node Eigen3::Eigen)

# Extended Kalman Filter
add_executable(extendedKalmanFilter_node src/extendedKalmanFilter_node.cpp)
ament_target_dependencies(extendedKalmanFilter_node rclcpp geometry_msgs nav_msgs)
target_link_libraries(extendedKalmanFilter_node Eigen3::Eigen)

# Particle Filter
add_executable(particleFilter_node src/particleFilter_node.cpp)
ament_target_dependencies(particleFilter_node rclcpp geometry_msgs nav_msgs visualization_msgs)
target_link_libraries(particleFilter_node Eigen3::Eigen)

# Trajectory & Noise
add_executable(infinity_node src/trajectory_node.cpp)
ament_target_dependencies(infinity_node rclcpp geometry_msgs visualization_msgs nav_msgs)

add_executable(noise_simulator_node src/noise_simulator_node.cpp)
ament_target_dependencies(noise_simulator_node rclcpp nav_msgs)

install(TARGETS
  kalmanFilter_node
  extendedKalmanFilter_node
  particleFilter_node
  infinity_node
  noise_simulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()