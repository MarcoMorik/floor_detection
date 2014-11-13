cmake_minimum_required(VERSION 2.8.3)
project(floor_detection)


find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  pcl_ros
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcl_snapshot src/pcl_snapshot.cpp)
target_link_libraries(pcl_snapshot ${catkin_LIBRARIES})
