cmake_minimum_required(VERSION 2.8.3)
project(line_control)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
)


catkin_package(
  CATKIN_DEPENDS nav_msgs roscpp
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(line_control_node
                src/line_control_node.cpp
                src/line_control.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(line_control_node
  ${catkin_LIBRARIES}
)
