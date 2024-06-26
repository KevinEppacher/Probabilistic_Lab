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
    realRaysPub = nh.advertise<visualization_msgs::MarkerArray>("real_laser_rays", 1);
    simRaysPub = nh.advertise<visualization_msgs::MarkerArray>("sim_laser_rays", 1);
    resampledParticlesPub = nh.advertise<geometry_msgs::PoseArray>("resampled_particles", 1);
    poseWithCovariancePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("poseWithCovariance", 1);

}

Visualizer::Visualizer::~Visualizer() 
{
 
}

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

void Visualizer::Visualizer::publishResampledParticles(geometry_msgs::PoseArray &poseArray, bool printPoseArray)
{
    poseArray.header.frame_id = "map";

    resampledParticlesPub.publish(poseArray);

    if (printPoseArray)
    {
        for (int i = 0; i < poseArray.poses.size(); ++i)
        {
            ROS_INFO("PoseArray[%d]: %f, %f, %f", i, poseArray.poses[i].position.x, poseArray.poses[i].position.y, tf::getYaw(poseArray.poses[i].orientation));
        }
    }
}

visualization_msgs::MarkerArray Visualizer::Visualizer::calcLaserRayArray(std::vector<Ray> &rays, double percent, visualization_msgs::Marker laserRay)
{
    visualization_msgs::MarkerArray laserRayArray;

    // Determine step size based on the desired percentage
    int step = std::max(1, static_cast<int>(rays.size() * percent / 100));

    for (int i = 0; i < rays.size(); i += step)
    {
        if (std::isfinite(rays[i].angle) && std::isfinite(rays[i].length))
        {
            laserRay.id = i;
            // Define the end point of the line based on the angle and length
            rays[i].end.x = rays[i].origin.x + rays[i].length * cos(rays[i].angle);
            rays[i].end.y = rays[i].origin.y + rays[i].length * sin(rays[i].angle);
            rays[i].end.z = rays[i].origin.z;

            laserRay.points.push_back(rays[i].origin);
            laserRay.points.push_back(rays[i].end);

            laserRayArray.markers.push_back(laserRay);
        }
    }
    return laserRayArray;
}


void Visualizer::Visualizer::publishRealRay(std::vector<Ray> &rays, double percent)
{
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
    laserRay.scale.x = 0.005; // Line width

    visualization_msgs::MarkerArray laserRayArray = calcLaserRayArray(rays, percent, laserRay);

    realRaysPub.publish(laserRayArray);
}

void Visualizer::Visualizer::publishSimRay(std::vector<Ray> &rays, double percent)
{
    visualization_msgs::Marker laserRay;
    laserRay.header.frame_id = "map";
    laserRay.header.stamp = ros::Time::now();
    laserRay.ns = "rays";
    laserRay.action = visualization_msgs::Marker::ADD;
    laserRay.type = visualization_msgs::Marker::LINE_STRIP;
    laserRay.pose.orientation.w = 1;
    laserRay.color.r = 0.0;
    laserRay.color.g = 0.0;
    laserRay.color.b = 1.0;
    laserRay.color.a = 1.0;
    laserRay.scale.x = 0.005; // Line width

    visualization_msgs::MarkerArray laserRayArray = calcLaserRayArray(rays, percent, laserRay);

    simRaysPub.publish(laserRayArray);
}

// void Visualizer::Visualizer::publishParticleRays(std::vector<Particle> particles, double percent)
// {
//     for(auto& particle : particles)
//     {
//         visualization_msgs::MarkerArray laserRayArray = calcLaserRayArray(particle.rays, percent);
//         realRaysPub.publish(laserRayArray);
//     }
// }

void Visualizer::Visualizer::clearMarkers()
{
    visualization_msgs::MarkerArray clearMarkerArray;
    visualization_msgs::Marker clearMarker;
    clearMarker.header.frame_id = "map";
    clearMarker.header.stamp = ros::Time::now();
    clearMarker.ns = "rays";                                    
    clearMarker.action = visualization_msgs::Marker::DELETEALL;
    clearMarkerArray.markers.push_back(clearMarker);
    // Publish the clear marker
    realRaysPub.publish(clearMarkerArray);
    simRaysPub.publish(clearMarkerArray);
    // ROS_INFO("Cleared all markers");
}

void Visualizer::Visualizer::publishPoseWithCovariance(const Particle &particle, bool printPoseWithCovariance)
{
    geometry_msgs::PoseWithCovarianceStamped poseWithCovarianceStamped;
    poseWithCovarianceStamped.header.stamp = ros::Time::now();
    poseWithCovarianceStamped.header.frame_id = "map"; // or the appropriate frame

    poseWithCovarianceStamped.pose.pose = particle.pose;

    std::fill(std::begin(poseWithCovarianceStamped.pose.covariance), std::end(poseWithCovarianceStamped.pose.covariance), 0.0);
    poseWithCovarianceStamped.pose.covariance[0] = particle.weight;

    // Debug log the size of the message
    ROS_DEBUG("Publishing PoseWithCovarianceStamped with size: %lu", sizeof(poseWithCovarianceStamped));

    poseWithCovariancePub.publish(poseWithCovarianceStamped);

    if (printPoseWithCovariance)
    {
        ROS_INFO("PoseWithCovariance: %f, %f, %f", 
                 poseWithCovarianceStamped.pose.pose.position.x, 
                 poseWithCovarianceStamped.pose.pose.position.y, 
                 tf::getYaw(poseWithCovarianceStamped.pose.pose.orientation));
    }
}
