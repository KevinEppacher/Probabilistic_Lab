#include "Visualizer.h"

Visualizer::Visualizer::Visualizer(ros::NodeHandle &nodehandler)
{
    nh = nodehandler;
    posePub = nh.advertise<geometry_msgs::Pose>("pose", 1);
    poseArrayPub = nh.advertise<geometry_msgs::PoseArray>("poseArray", 100);
    mapPub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
    laserScanPub = nh.advertise<sensor_msgs::LaserScan>("laserScan", 1);
    odomPub = nh.advertise<nav_msgs::Odometry>("odom", 1);
    markerPub = nh.advertise<visualization_msgs::Marker>("marker", 1);
    poseArrayMotionModel = nh.advertise<geometry_msgs::PoseArray>("poseArrayMotionModel", 1);
}

Visualizer::Visualizer::~Visualizer() {}

void Visualizer::Visualizer::publishPose(const geometry_msgs::Pose &pose, bool printPose)
{
    posePub.publish(pose);

    if (printPose)
    {
        ROS_INFO("Pose: %f, %f, %f", pose.position.x, pose.position.y, pose.orientation.z);
    }
}

void Visualizer::Visualizer::publishPoseArray(geometry_msgs::PoseArray &poseArray, bool printPoseArray)
{
    poseArray.header.frame_id = "map"; // der relevante Frame, typischerweise "map" oder "odom"

    poseArrayPub.publish(poseArray);

    if (printPoseArray)
    {
        for (int i = 0; i < poseArray.poses.size(); ++i)
        {
            ROS_INFO("PoseArray[%d]: %f, %f, %f", i, poseArray.poses[i].position.x, poseArray.poses[i].position.y, tf::getYaw(poseArray.poses[i].orientation));
        }
    }
}

void Visualizer::Visualizer::publishMap(const nav_msgs::OccupancyGrid &map, bool printMap)
{
    mapPub.publish(map);

    if (printMap)
    {
        ROS_INFO("Map: %d, %d, %f", map.info.width, map.info.height, map.info.resolution);
    }
}

void Visualizer::Visualizer::publishLaserScan(const sensor_msgs::LaserScan &laserScan, bool printLaserScan)
{
    laserScanPub.publish(laserScan);

    if (printLaserScan)
    {
        ROS_INFO("LaserScan: %f, %f, %f", laserScan.angle_min, laserScan.angle_max, laserScan.range_max);
    }
}

void Visualizer::Visualizer::publishOdom(const nav_msgs::Odometry &odom, bool printOdom)
{
    odomPub.publish(odom);

    if (printOdom)
    {
        ROS_INFO("Odom: %f, %f, %f", odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.orientation.z);
    }
}

void Visualizer::Visualizer::publishMarker(const visualization_msgs::Marker &marker, bool printMarker)
{
    markerPub.publish(marker);

    if (printMarker)
    {
        ROS_INFO("Marker: %d, %d, %d", marker.id, marker.type, marker.action);
    }
}

void Visualizer::Visualizer::publishPoseArrayFromMotionModel(geometry_msgs::PoseArray &poseArray, bool printPoseArray)
{
    poseArray.header.frame_id = "map"; // der relevante Frame, typischerweise "map" oder "odom"

    poseArrayMotionModel.publish(poseArray);

    if (printPoseArray)
    {
        for (int i = 0; i < poseArray.poses.size(); ++i)
        {
            ROS_INFO("PoseArray[%d]: %f, %f, %f", i, poseArray.poses[i].position.x, poseArray.poses[i].position.y, tf::getYaw(poseArray.poses[i].orientation));
        }
    }
}
