#include "Motion_Model.h"

MotionModel::MotionModel(ros::NodeHandle &nh) : gen(std::random_device{}())
{
    nh.getParam("alpha1", alpha1);
    nh.getParam("alpha2", alpha2);
    nh.getParam("alpha3", alpha3);
    nh.getParam("alpha4", alpha4);
    nh.getParam("alpha5", alpha5);
    nh.getParam("alpha6", alpha6);
    nh.getParam("loop_rate", loopRate);

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

geometry_msgs::Pose MotionModel::sampleMotionModel(geometry_msgs::Twist motionCommand, geometry_msgs::Pose prevPose, const double& dt)
{
    theta = tf::getYaw(prevPose.orientation);
    theta = normalize_angle_positive(theta);

    double x = prevPose.position.x;
    double y = prevPose.position.y;
    double newX, newY;
    geometry_msgs::Pose newPose;

    v = motionCommand.linear.x;
    w = motionCommand.angular.z;

    v_hat = v + sample(alpha1 * std::abs(v) + alpha2 * std::abs(w));

    w_hat = w + sample(alpha3 * std::abs(v) + alpha4 * std::abs(w));

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

    return newPose;
}

double MotionModel::sample(double std_dev)
{
    std::normal_distribution<> dist(0.0, std::sqrt(std_dev));
    return dist(gen);
}

double MotionModel::getTimeDifference()
{
    dt = 1 / loopRate;
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