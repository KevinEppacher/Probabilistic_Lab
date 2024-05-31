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
    geometry_msgs::Twist sampledMotion;

    theta = prevPose.orientation.z;
    ROS_INFO("Theta: %f", theta);

    dt = getTimeDifference();
    
    v = motionCommand.linear.x;
    w = motionCommand.angular.z;

    v_hat = v_hat + sample(alpha1 * std::abs(v) + alpha2 * std::abs(w));
    w_hat = w_hat + sample(alpha3 * std::abs(v) + alpha4 * std::abs(w));
    gamma_hat = sample(alpha5 * std::abs(v) + alpha6 * std::abs(w));

    sampledMotion.linear.x = sampledMotion.linear.x - ( v_hat * sin(theta)) / w_hat + ( v_hat * sin(theta + w_hat * dt)) / w_hat;
    sampledMotion.linear.y = sampledMotion.linear.y + ( v_hat * cos(theta)) / w_hat - ( v_hat * cos(theta + w_hat * dt)) / w_hat;
    sampledMotion.angular.z = sampledMotion.angular.z + w_hat * dt + gamma_hat * dt;

    return sampledMotion;
}

double MotionModel::sample(double std_dev) 
{
    static std::random_device rd;
    static std::default_random_engine generator(rd());
    std::normal_distribution<double> distribution(0.0, std_dev);
    return distribution(generator);
}

double MotionModel::getTimeDifference()
{
    static ros::Time prevTime = ros::Time::now();
    ros::Time currentTime = ros::Time::now();
    double dt = (currentTime - prevTime).toSec();
    prevTime = currentTime;
    return dt;
}