cmake_minimum_required(VERSION 3.8)
project(hokuyo_navigation2)

if(CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(jsk_rviz_plugins REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Add your new executable here
add_executable(gnss_lio_debug src/gnss_lio_debug.cpp)
add_executable(pointcloud_transform_for_loc src/pointcloud_transform_for_loc.cpp)
add_executable(odom_to_tf src/odom_to_tf.cpp)
add_executable(odom_relative src/odom_relative.cpp)
add_executable(cmdvel_stopper src/cmdvel_stopper.cpp)

ament_target_dependencies(gnss_lio_debug
  rclcpp
  jsk_rviz_plugins
  std_msgs
  sensor_msgs
)

ament_target_dependencies(pointcloud_transform_for_loc
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
)

ament_target_dependencies(odom_to_tf
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  Eigen3
)

ament_target_dependencies(odom_relative
  rclcpp
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(cmdvel_stopper
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# jsk_rviz_plugins のインクルードディレクトリを追加
target_include_directories(gnss_lio_debug PUBLIC
  ${jsk_rviz_plugins_INCLUDE_DIRS}
)

# 新しいノードのインクルードディレクトリを追加
target_include_directories(pointcloud_transform_for_loc PUBLIC
  ${pcl_conversions_INCLUDE_DIRS}
)
target_include_directories( odom_to_tf PUBLIC
)
target_include_directories( odom_relative PUBLIC
)

install(TARGETS
  gnss_lio_debug
  pointcloud_transform_for_loc
  odom_to_tf
  odom_relative
  cmdvel_stopper
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY urdf config launch rviz src scripts map waypoints
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/coordinator.sh
  DESTINATION lib/${PROJECT_NAME})


ament_export_dependencies(
  rclcpp
  jsk_rviz_plugins
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  pcl_conversions
)

ament_package()