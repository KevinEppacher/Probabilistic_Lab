#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <random>
#include <tf/transform_datatypes.h>

class MotionModel 
{
public:
    MotionModel(ros::NodeHandle& nh);
    MotionModel();
    ~MotionModel();
    geometry_msgs::Twist sampleMotionModel(geometry_msgs::Twist motionCommand, geometry_msgs::Pose prevPose);
    double sample(double std_dev);
    double getTimeDifference();


private:
    double alpha1, alpha2, alpha3, alpha4, alpha5, alpha6;
    double v_hat, w_hat, v, w, gamma_hat, theta, dt;

};

#endif // MOTION_MODEL_H
