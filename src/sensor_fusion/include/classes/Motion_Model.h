#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class MotionModel 
{
public:
    MotionModel(ros::NodeHandle& nh);
    MotionModel();
    ~MotionModel();
    geometry_msgs::Twist sampleMotionModel(geometry_msgs::Twist motionCommand, geometry_msgs::Pose prevPose);

private:
    double alpha1, alpha2, alpha3, alpha4, alpha5, alpha6;

};

#endif // MOTION_MODEL_H
