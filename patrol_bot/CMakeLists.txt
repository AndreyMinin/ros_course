cmake_minimum_required(VERSION 2.8.3)
project(patrol_bot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  tf
)
add_definitions(-std=c++11)


catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(patrol_bot_node src/patrol_bot_node.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(patrol_bot_node
  ${catkin_LIBRARIES}
)
