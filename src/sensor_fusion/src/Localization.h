// Localization.h
#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

class Localization
{
private:
    ros::NodeHandle nh_;  // Node handle to manage communication with ROS
    ros::Subscriber odom_sub_;  // Subscriber for odometry

    // Callback function to handle incoming odometry messages
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

public:
    Localization();
    ~Localization();
};


class ParticleFilter
{
private:


public:
    ParticleFilter();
    ~ParticleFilter();
};