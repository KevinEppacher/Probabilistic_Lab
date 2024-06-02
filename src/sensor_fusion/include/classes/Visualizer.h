#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h> 
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include "../structs/Particle.h"

namespace Visualizer
{
    class Visualizer
    {
    public:
        explicit Visualizer(ros::NodeHandle &nodehandler);
        Visualizer();
        virtual ~Visualizer();

        void publishPose(const geometry_msgs::Pose &pose, bool printPose = false);
        void publishPoseArray(geometry_msgs::PoseArray &poseArray, bool printPoseArray = false);
        void publishMap(const nav_msgs::OccupancyGrid &map, bool printMap = false);
        void publishLaserScan(const sensor_msgs::LaserScan &laserScan, bool printLaserScan = false);
        void publishOdom(const nav_msgs::Odometry &odom, bool printOdom = false);
        void publishMarker(const visualization_msgs::Marker &marker, bool printMarker = false);
        void publishPoseArray(const std::vector<Particle>& particles);

    private:
        ros::NodeHandle nh;
        ros::Publisher posePub;
        ros::Publisher poseArrayPub;
        ros::Publisher mapPub;
        ros::Publisher laserScanPub;
        ros::Publisher odomPub;
        ros::Publisher markerPub;
    };
} // namespace Visualizer


#endif // VISUALIZER_H