#pragma once

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>

void scan_to_map(const sensor_msgs::LaserScan& scan, nav_msgs::OccupancyGrid& map, const tf::Transform& transform);


