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

    // ROS_INFO(" Current Pose: %f, %f, %f", currentPose.position.x, currentPose.position.y, theta * 180.0 / M_PI);

    dt = getTimeDifference();

    v = motionCommand.linear.x;
    w = motionCommand.angular.z;

    // ROS_INFO("v: %f", v);
    // ROS_INFO("w: %f", w);

    v_hat = v + sample(alpha1 * std::abs(v) + alpha2 * std::abs(w));
    // ROS_INFO("v_hat: %f", v_hat);

    w_hat = w + sample(alpha3 * std::abs(v) + alpha4 * std::abs(w));
    // ROS_INFO("w_hat: %f", w_hat);

    gamma_hat = sample(alpha5 * std::abs(v) + alpha6 * std::abs(w));

    // ROS_INFO(" Current Pose: %f, %f, %f", currentPose.position.x, currentPose.position.y, theta * 180.0 / M_PI);

    if(fabs(w_hat) > 1e-5) // Avoid division by zero
    {
        currentPose.position.x = currentPose.position.x - (v_hat * sin(theta) / w_hat) + (v_hat * sin(theta + w_hat * dt) / w_hat);
        currentPose.position.y = currentPose.position.y + (v_hat * cos(theta) / w_hat) - (v_hat * cos(theta + w_hat * dt) / w_hat);
    }
    else
    {
        // When Robot is moving in a straight line
        currentPose.position.x = currentPose.position.x + v_hat * cos(theta) * dt;
        currentPose.position.y = currentPose.position.y + v_hat * sin(theta) * dt;
    }

    theta = theta + w_hat * dt + gamma_hat * dt;
    currentPose.orientation = tf::createQuaternionMsgFromYaw(normalize_angle_positive(theta));

    // ROS_INFO("Sampled Pose: %f, %f, %f", currentPose.position.x, currentPose.position.y, tf::getYaw(currentPose.orientation));

    return currentPose;
}

double MotionModel::sample(double std_dev)
{
    // static std::random_device rd;
    // static std::default_random_engine generator(rd());
    // std::normal_distribution<double> distribution(0.0, std_dev);
    // return distribution(generator);

    std::normal_distribution<> dist(0.0, std::sqrt(std_dev));
    return dist(gen);

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