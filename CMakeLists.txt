cmake_minimum_required(VERSION 3.22)
project(pointcloud_to_grid)

# 使用 C++14 或更高
set(CMAKE_CXX_STANDARD 14)

# 找 ROS 2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

include_directories(include ${PCL_INCLUDE_DIRS})

add_executable(pointcloud_to_occupancy_grid_node src/pointcloud_to_occupancy_grid_node.cpp)
ament_target_dependencies(pointcloud_to_occupancy_grid_node
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions
)

target_link_libraries(pointcloud_to_occupancy_grid_node ${PCL_LIBRARIES})

install(TARGETS pointcloud_to_occupancy_grid_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

