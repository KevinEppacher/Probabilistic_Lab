#include "Motion_Model.h"

MotionModel::MotionModel(ros::NodeHandle &nh)
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

geometry_msgs::Pose MotionModel::sampleMotionModel(geometry_msgs::Twist motionCommand, geometry_msgs::Pose currentPose)
{
    theta = tf::getYaw(currentPose.orientation);
    theta = normalize_angle_positive(theta);
    // ROS_INFO("Theta: %f degrees", theta * 180.0 / M_PI);

    dt = getTimeDifference();

    v = motionCommand.linear.x;
    w = motionCommand.angular.z;

    // ROS_INFO("v: %f", v);
    // ROS_INFO("w: %f", w);

    v_hat = v_hat + sample(alpha1 * std::abs(v) + alpha2 * std::abs(w));
    // ROS_INFO("v_hat: %f", v_hat);

    w_hat = w_hat + sample(alpha3 * std::abs(v) + alpha4 * std::abs(w));
    // ROS_INFO("w_hat: %f", w_hat);

    gamma_hat = sample(alpha5 * std::abs(v) + alpha6 * std::abs(w));

    sampledPose.position.x = currentPose.position.x - (v_hat * sin(theta)) / w_hat + (v_hat * sin(theta + w_hat * dt)) / w_hat;
    sampledPose.position.y = currentPose.position.y + (v_hat * cos(theta)) / w_hat - (v_hat * cos(theta + w_hat * dt)) / w_hat;
    theta = theta + w_hat * dt + gamma_hat * dt;
    sampledPose.orientation = tf::createQuaternionMsgFromYaw(theta);

    ROS_INFO("Sampled Pose: %f, %f, %f", sampledPose.position.x, sampledPose.position.y, tf::getYaw(sampledPose.orientation));

    return sampledPose;
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

double MotionModel::normalize_angle_positive(double angle)
{
    return fmod(fmod(angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
}