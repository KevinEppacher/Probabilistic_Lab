#include "Visualizer.h"

Visualizer::Visualizer::Visualizer(ros::NodeHandle &nodehandler) : nh(nodehandler)
{
    posePub = nh.advertise<geometry_msgs::Pose>("pose", 1);
    poseArrayPub = nh.advertise<geometry_msgs::PoseArray>("poseArray", 1);
    mapPub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
    laserScanPub = nh.advertise<sensor_msgs::LaserScan>("laserScan", 1);
    odomPub = nh.advertise<nav_msgs::Odometry>("odom", 1);
    markerPub = nh.advertise<visualization_msgs::Marker>("marker", 1);
    poseArrayMotionModelPub = nh.advertise<geometry_msgs::PoseArray>("poseArrayMotionModel", 1);
    initialParticlesPub = nh.advertise<geometry_msgs::PoseArray>("initialParticles", 1);
    raysPub = nh.advertise<visualization_msgs::MarkerArray>("laser_rays", 1);
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
    poseArray.header.frame_id = "map";

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

    poseArrayMotionModelPub.publish(poseArray);

    if (printPoseArray)
    {
        for (int i = 0; i < poseArray.poses.size(); ++i)
        {
            ROS_INFO("PoseArray[%d]: %f, %f, %f", i, poseArray.poses[i].position.x, poseArray.poses[i].position.y, tf::getYaw(poseArray.poses[i].orientation));
        }
    }
}

void Visualizer::Visualizer::publishInitialParticles(geometry_msgs::PoseArray &poseArray, bool printPoseArray)
{
    poseArray.header.frame_id = "map";
    
    initialParticlesPub.publish(poseArray);

    if (printPoseArray)
    {
        for (int i = 0; i < poseArray.poses.size(); ++i)
        {
            ROS_INFO("PoseArray[%d]: %f, %f, %f", i, poseArray.poses[i].position.x, poseArray.poses[i].position.y, tf::getYaw(poseArray.poses[i].orientation));
        }
    }
}


void Visualizer::Visualizer::publishRay(std::vector<Ray>& rays)
{
    visualization_msgs::MarkerArray laserRayArray;
    visualization_msgs::Marker laserRay;
    laserRay.header.frame_id = "map";
    laserRay.header.stamp = ros::Time::now();
    laserRay.ns = "rays";
    laserRay.action = visualization_msgs::Marker::ADD;
    laserRay.type = visualization_msgs::Marker::LINE_STRIP;
    laserRay.pose.orientation.w = 1;
    laserRay.color.r = 1.0;
    laserRay.color.g = 0.0;
    laserRay.color.b = 0.0;
    laserRay.color.a = 1.0;

    for(int i = 0; i < rays.size(); ++i)
    {
        laserRay.id = i;

        laserRay.scale.x = 0.01; // Line width



        // Define the end point of the line based on the angle and length
        rays[i].end.x = rays[i].origin.x + rays[i].length * cos(rays[i].angle);
        rays[i].end.y = rays[i].origin.y + rays[i].length * sin(rays[i].angle);
        rays[i].end.z = rays[i].origin.z;

        laserRay.points.push_back(rays[i].origin);
        laserRay.points.push_back(rays[i].end);

        laserRayArray.markers.push_back(laserRay);

    }

    raysPub.publish(laserRayArray);





    // visualization_msgs::MarkerArray marker_array;

    // for (int i = 0; i < rays.size(); ++i)
    // {
    //     visualization_msgs::Marker line;
    //     line.header.frame_id = "map";
    //     line.header.stamp = ros::Time::now();
    //     line.ns = "lines";
    //     line.action = visualization_msgs::Marker::ADD;
    //     line.pose.orientation.w = 1.0;

    //     line.id = i;
    //     line.type = visualization_msgs::Marker::LINE_STRIP;

    //     line.scale.x = 0.1; // Line width

    //     line.color.b = 1.0;
    //     line.color.a = 1.0;

    //     // Define the end point of the line based on the angle and length
    //     geometry_msgs::Point end_point;
    //     end_point.x = rays[i].origin.x + rays[i].length * cos(rays[i].angle);
    //     end_point.y = rays[i].origin.y + rays[i].length * sin(rays[i].angle);
    //     end_point.z = rays[i].origin.z;

    //     line.points.push_back(rays[i].origin);
    //     line.points.push_back(end_point);

    //     marker_array.markers.push_back(line);
    // }

    // raysPub.publish(marker_array);
}
