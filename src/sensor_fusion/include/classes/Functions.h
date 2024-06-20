//Include Libraries
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <random>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


// std::vector<std::pair<float, float>> findFreeCells(const nav_msgs::OccupancyGrid &map)
// {
//     std::vector<std::pair<float, float>> free_cells;
//     for (unsigned int y = 0; y < map.info.height; y++)
//     {
//         for (unsigned int x = 0; x < map.info.width; x++)
//         {
//             int index = x + y * map.info.width;
//             if (map.data[index] == 0)
//             { // 0 bedeutet frei
//                 float world_x = map.info.origin.position.x + (x + 0.5) * map.info.resolution;
//                 float world_y = map.info.origin.position.y + (y + 0.5) * map.info.resolution;
//                 free_cells.emplace_back(world_x, world_y);
//             }
//         }
//     }
//     return free_cells;
// }

