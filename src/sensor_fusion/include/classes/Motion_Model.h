#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <random>
#include <tf/transform_datatypes.h>
#include <cmath>

// Dynamic Reconfigure
#include <dynamic_reconfigure/server.h>
#include <mutex>

// Dynamic Reconfigure Motion Model Parameters
#include <sensor_fusion/MotionModelConfig.h>

#include "Functions.h"

class MotionModel 
{
public:
    MotionModel(ros::NodeHandle& nh);
    MotionModel();
    ~MotionModel();
    geometry_msgs::Pose sampleMotionModel(geometry_msgs::Twist motionCommand, geometry_msgs::Pose prevPose);
    double sample(double std_dev);
    double getTimeDifference();
    double normalize_angle_positive(double angle);
    void configCallback(sensor_fusion::MotionModelConfig &config, uint32_t level);

private:
    double alpha1, alpha2, alpha3, alpha4, alpha5, alpha6;
    double v_hat, w_hat, v, w, gamma_hat, theta, dt;
    mutable std::mt19937 gen;
    double loopRate;

    dynamic_reconfigure::Server<sensor_fusion::MotionModelConfig>* server;
    dynamic_reconfigure::Server<sensor_fusion::MotionModelConfig>::CallbackType f;

    ros::Time prevTime, currentTime;

};

#endif // MOTION_MODEL_H
