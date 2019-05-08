#include <gtest/gtest.h>
#include <math.h>

#include "simple_map/scan_to_map.h"

TEST(TestExample, MathOperations){
  EXPECT_EQ(pow(2, 2), 4);
  EXPECT_TRUE(cos(M_PI/45) < 1.0);
  EXPECT_FLOAT_EQ(sin(M_PI/6), 0.5);
}


TEST(ScanToMap, SimpleMap) {
    sensor_msgs::LaserScan scan;
    // set only  point is zero
    scan.ranges.push_back(0.05);
    scan.angle_min = 0;
    scan.angle_max = 0;
    scan.angle_increment = 0;

   nav_msgs::OccupancyGrid map;

   map.info.height = 100;
   map.info.width = 100;
   map.info.resolution = 0.1;
   map.info.origin.position.x = - (map.info.width * map.info.resolution) / 2;
   map.info.origin.position.y = - (map.info.height * map.info.resolution) / 2;

   // изменяем размер вектора, который является хранилищем данных карты, и заполняем его значением (-1) - неизвестное значение
   map.data.resize(map.info.height*map.info.width, -1);

   scan_to_map(scan, map, tf::Transform::getIdentity());
   EXPECT_EQ(map.data[map.info.width / 2 + (map.info.height / 2 - 1) * map.info.width], 100);

}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
