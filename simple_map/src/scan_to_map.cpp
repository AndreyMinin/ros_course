#include "simple_map/scan_to_map.h"
#include <ros/ros.h>


void scan_to_map(const sensor_msgs::LaserScan& scan, nav_msgs::OccupancyGrid& map, const tf::Transform& transform) {
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
        double angle = scan.angle_min + scan.angle_increment * i;
        tf::Vector3 v = transform(tf::Vector3(scan.ranges[i]*cos(angle), scan.ranges[i]*sin(angle), 0));
        v -= tf::Vector3(map.info.origin.position.x, map.info.origin.position.y, 0);
        if (v.x() >= 0 && v.y() >= 0 &&
                v.x() < map.info.resolution * map.info.width && v.y() < map.info.resolution * map.info.height) {
            std::size_t x = v.x() / map.info.resolution;
            std::size_t y = v.y() / map.info.resolution;
        }
    }
}
