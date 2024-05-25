#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

namespace Communication {

class Communication {
public:
    explicit Communication(ros::NodeHandle& nodehandler);
    virtual ~Communication();

protected:
    ros::NodeHandle nh;
};

class Publisher : public Communication {
public:
    Publisher(ros::NodeHandle& nodehandler);
    void publishTwist(const geometry_msgs::Twist& msg);
    void publishPose(const geometry_msgs::Pose& msg);

private:
    ros::Publisher twistPub;
    ros::Publisher posePub;
};

class Subscriber : public Communication {
public:
    Subscriber(ros::NodeHandle& nodehandler);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

private:
    ros::Subscriber odomSub;


};

}
#endif // COMMUNICATION_H
