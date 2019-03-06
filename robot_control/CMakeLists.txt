cmake_minimum_required(VERSION 2.8.3)
project(robot_control)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(
  robot_control src/robot_control.cpp)

target_link_libraries(
  robot_control ${catkin_LIBRARIES}
)

