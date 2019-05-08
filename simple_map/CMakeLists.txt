cmake_minimum_required(VERSION 2.8.3)
project(simple_map)

add_compile_options(-std=c++11)


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
   INCLUDE_DIRS include
   LIBRARIES simple_map_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(simple_map_lib include/simple_map/scan_to_map.h src/scan_to_map.cpp)


## Declare a C++ executable
add_executable(simple_map_node src/simple_map.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(simple_map_node
  ${catkin_LIBRARIES}
)

catkin_add_gtest(scan_to_map_test test/scan_to_map_test.cpp)
target_link_libraries(scan_to_map_test simple_map_lib ${catkin_LIBRARIES})
