#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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

// Custom Structs
#include "../structs/Particle.h"
#include "../structs/Ray.h"

namespace Visualizer
{
    class Visualizer
    {
    public:
        explicit Visualizer(ros::NodeHandle &nodehandler);
        Visualizer();
        virtual ~Visualizer();
        // Cummon
        void publishPose(const geometry_msgs::Pose &pose, bool printPose = false);
        void publishMap(const nav_msgs::OccupancyGrid &map, bool printMap = false);
        void publishLaserScan(const sensor_msgs::LaserScan &laserScan, bool printLaserScan = false);
        void publishOdom(const nav_msgs::Odometry &odom, bool printOdom = false);
        void publishMarker(const visualization_msgs::Marker &marker, bool printMarker = false);
        void publishPoseArray(geometry_msgs::PoseArray &poseArray, bool printPoseArray = false);
        // Custom Pose Arrays(delete these Methdos if not needed)
        void publishInitialParticles(geometry_msgs::PoseArray &poseArray, bool printPoseArray);
        void publishPoseArrayFromMotionModel(geometry_msgs::PoseArray &poseArray, bool printPoseArray = false);
        void publishRealRay(std::vector<Ray>& rays, double percent);
        void publishSimRay(std::vector<Ray>& rays, double percent);
        void publishParticleRays(std::vector<Particle> particles, double percent);
        void clearMarkers();
        void publishResampledParticles(geometry_msgs::PoseArray &poseArray, bool printPoseArray);


    private:
        visualization_msgs::MarkerArray calcLaserRayArray(std::vector<Ray> &rays, double percent, visualization_msgs::Marker laserRay);


        ros::NodeHandle nh;
        ros::Publisher posePub;
        ros::Publisher poseArrayPub;
        ros::Publisher mapPub;
        ros::Publisher laserScanPub;
        ros::Publisher odomPub;
        ros::Publisher markerPub;

        // Custom Pose Arrays(delete these Methdos if not needed)
        ros::Publisher poseArrayMotionModelPub, initialParticlesPub, resampledParticlesPub;
        ros::Publisher realRaysPub, simRaysPub;
    };
} // namespace Visualizer


#endif // VISUALIZER_H