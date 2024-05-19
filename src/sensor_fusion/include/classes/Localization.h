#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "Particle_Filter.h"
#include "Motion_Model.h"


class Localization 
{
public:
    Localization();
    ~Localization();

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

private:
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
};

#endif // LOCALIZATION_H
