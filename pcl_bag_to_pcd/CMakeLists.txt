cmake_minimum_required(VERSION 3.5)
project(pcl_bag_to_pcd)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_bag_to_pcd src/pcl_bag_to_pcd.cpp)
target_link_libraries(pcl_bag_to_pcd ${PCL_LIBRARIES})

ament_target_dependencies(pcl_bag_to_pcd
  rclcpp
  sensor_msgs
  pcl_conversions
  rosbag2_cpp
  PCL
)

install(
  TARGETS pcl_bag_to_pcd
  DESTINATION lib/${PROJECT_NAME}
  PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}/
)

ament_export_dependencies(
  geometry_msgs
  rclcpp
  sensor_msgs
  PCL
  pcl_conversions
)

ament_package()
