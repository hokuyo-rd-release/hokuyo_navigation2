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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Add your new executable here
add_executable(gnss_lio_debug src/gnss_lio_debug.cpp)

ament_target_dependencies(gnss_lio_debug
  rclcpp
  jsk_rviz_plugins
  std_msgs
  sensor_msgs
)

# jsk_rviz_plugins のインクルードディレクトリを追加
target_include_directories(gnss_lio_debug PUBLIC
  ${jsk_rviz_plugins_INCLUDE_DIRS}
)

install(TARGETS
  gnss_lio_debug
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY urdf config launch rviz src scripts map waypoints
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/expo_start.sh
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rclcpp jsk_rviz_plugins std_msgs sensor_msgs)

ament_package()