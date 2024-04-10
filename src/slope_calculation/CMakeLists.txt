cmake_minimum_required(VERSION 3.0.2)
project(slope_calculation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
)

find_package(PCL REQUIRED)

include_directories(
  include/slope_calculation
  ${catkin_INCLUDE_DIRS}
)

add_executable(slope_calculation src/slope_calculation.cpp src/slope_calculation_node.cpp src/least_square.cpp)

target_link_libraries(slope_calculation ${catkin_LIBRARIES} ${PCL_LIBRARIES})