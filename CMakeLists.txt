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
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Add your new executable here
add_executable(gnss_lio_debug src/gnss_lio_debug.cpp)
add_executable(pcd_tf_transform src/pcd_tf_transform.cpp)
add_executable(odom_frame_changer src/odom_frame_changer.cpp)

ament_target_dependencies(gnss_lio_debug
  rclcpp
  jsk_rviz_plugins
  std_msgs
  sensor_msgs
)

ament_target_dependencies(pcd_tf_transform
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
)

ament_target_dependencies(odom_frame_changer
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  Eigen3
)

# jsk_rviz_plugins のインクルードディレクトリを追加
target_include_directories(gnss_lio_debug PUBLIC
  ${jsk_rviz_plugins_INCLUDE_DIRS}
)

# 新しいノードのインクルードディレクトリを追加
target_include_directories(pcd_tf_transform PUBLIC
  ${pcl_conversions_INCLUDE_DIRS}
)
target_include_directories( odom_frame_changer PUBLIC
  "/usr/local/include/eigen3"
)

install(TARGETS
  gnss_lio_debug
  pcd_tf_transform
  odom_frame_changer
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
  tf2_sensor_msgs
  pcl_conversions
)

ament_package()