cmake_minimum_required(VERSION 3.0.2)
project(expo_wizurg)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  pcl_ros
  sensor_msgs
)

catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

add_subdirectory(simple_lio_localization)

include_directories(
  simple_lio_localization/include
  simple_lio_localization/thirdparty/small_gicp/include
  ${catkin_INCLUDE_DIRS}
  include
    ${PCL_INCLUDE_DIRS}
)

add_executable(pcd_to_pgm_node src/pcd2pgm.cpp)
target_link_libraries(pcd_to_pgm_node ${catkin_LIBRARIES} ${PCL_LIBRARIES}  )

add_executable(simple_fastlio_localization_node src/localization_node.cpp)
add_executable(pose_to_tf src/pose_to_tf.cpp)

target_link_libraries(simple_fastlio_localization_node
  ${catkin_LIBRARIES}
  simple_lio_localization
  Eigen3::Eigen
)
target_link_libraries(pose_to_tf ${catkin_LIBRARIES})

add_executable(controller_setup src/controller_setup.cpp)
target_link_libraries(controller_setup ${catkin_LIBRARIES})

add_executable(gnss_logger_node src/gnss_logger_node.cpp)
target_link_libraries(gnss_logger_node ${catkin_LIBRARIES})

add_executable(joy_to_twist src/joy_to_twist.cpp)
target_link_libraries(joy_to_twist ${catkin_LIBRARIES})