#include "Motion_Model.h"

MotionModel::MotionModel(ros::NodeHandle &nh) : gen(std::random_device{}())
{
    nh.getParam("alpha1", alpha1);
    nh.getParam("alpha2", alpha2);
    nh.getParam("alpha3", alpha3);
    nh.getParam("alpha4", alpha4);
    nh.getParam("alpha5", alpha5);
    nh.getParam("alpha6", alpha6);

    //     // Initialize dynamic reconfigure server
    server = new dynamic_reconfigure::Server<sensor_fusion::MotionModelConfig>(nh.getNamespace() + "/motion_model");

    f = boost::bind(&MotionModel::configCallback, this, _1, _2);

    server->setCallback(f);
}

MotionModel::MotionModel() {}

MotionModel::~MotionModel()
{
    delete server;
}

geometry_msgs::Pose MotionModel::sampleMotionModel(geometry_msgs::Twist motionCommand, geometry_msgs::Pose prevPose)
{
    theta = tf::getYaw(prevPose.orientation);
    theta = normalize_angle_positive(theta);

    double x = prevPose.position.x;
    double y = prevPose.position.y;
    double newX, newY;
    geometry_msgs::Pose newPose;
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


    if (fabs(w_hat) > 1e-5) // Avoid division by zero
    {
        newX = x - (v_hat * sin(theta) / w_hat) + (v_hat * sin(theta + w_hat * dt) / w_hat);
        newY = y + (v_hat * cos(theta) / w_hat) - (v_hat * cos(theta + w_hat * dt) / w_hat);
    }
    else
    {
        // When Robot is moving in a straight line
        newX = x + v_hat * cos(theta) * dt;
        newY = y + v_hat * sin(theta) * dt;
    }

    theta = theta + w_hat * dt + gamma_hat * dt;

    newPose.position.x = newX;
    newPose.position.y = newY;  
    newPose.orientation = tf::createQuaternionMsgFromYaw(normalize_angle_positive(theta));

    // ROS_INFO("Sampled Pose: %f, %f, %f", currentPose.position.x, currentPose.position.y, tf::getYaw(currentPose.orientation));

    // ROS_WARN(" Current Pose: %f, %f, %f", currentPose.position.x, currentPose.position.y, theta * 180.0 / M_PI);

    return newPose;
}

double MotionModel::sample(double std_dev)
{
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

void MotionModel::configCallback(sensor_fusion::MotionModelConfig &config, uint32_t level)
{
    alpha1 = config.alpha1;
    alpha2 = config.alpha2;
    alpha3 = config.alpha3;
    alpha3 = config.alpha4;
    alpha5 = config.alpha5;
    alpha6 = config.alpha6;

    ROS_INFO("Reconfigure Request: alpha1=%f, alpha2=%f, alpha3=%f, alpha4=%f, alpha5=%f, alpha6=%f",
             alpha1, alpha2, alpha3, alpha4, alpha5, alpha6);
}