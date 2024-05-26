#include "Motion_Model.h"

MotionModel::MotionModel(ros::NodeHandle& nh) 
{
    nh.getParam("alpha1", alpha1);
    nh.getParam("alpha2", alpha2);
    nh.getParam("alpha3", alpha3);
    nh.getParam("alpha4", alpha4);
    nh.getParam("alpha5", alpha5);
    nh.getParam("alpha6", alpha6);
}

MotionModel::MotionModel() {}

MotionModel::~MotionModel() {}

geometry_msgs::Twist MotionModel::sampleMotionModel(geometry_msgs::Twist motionCommand, geometry_msgs::Pose prevPose)
{
    geometry_msgs::Twist sampledMotionCommands, sampledMotion;
    
    sampledMotionCommands.linear.x = motionCommand.linear.x + alpha1;
    sampledMotionCommands.angular.z = motionCommand.angular.z + alpha2;

    // std::cout << "Sampled Motion Commands: " << sampledMotionCommands.linear.x << ", " << sampledMotionCommands.angular.z << std::endl;

    return sampledMotion;
}