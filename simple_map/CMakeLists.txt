cmake_minimum_required(VERSION 2.8.3)
project(simple_map)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  tf
)

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
add_executable(simple_map_node src/simple_map.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(simple_map_node
  ${catkin_LIBRARIES}
)
