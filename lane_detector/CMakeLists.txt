cmake_minimum_required(VERSION 2.8.3)
project(lane_detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

# System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)



catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(lane_detector src/lane_detector.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(lane_detector
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
    )


