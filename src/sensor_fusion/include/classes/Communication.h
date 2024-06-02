#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

namespace Communication {

class Communication {
public:
    explicit Communication(ros::NodeHandle& nodehandler);
    Communication();
    virtual ~Communication();

protected:
    ros::NodeHandle nh;
};

class Publisher : public Communication {
public:
    Publisher(ros::NodeHandle& nodehandler);
    void publishTwist(const geometry_msgs::Twist& msg);
    void publishPose(const geometry_msgs::Pose& msg, bool printPose = false);

private:
    ros::Publisher twistPub;
    ros::Publisher posePub;
};

class Subscriber : public Communication {
public:
    Subscriber(ros::NodeHandle& nodehandler);
    // Odometry Subscriber
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    nav_msgs::Odometry getOdom(bool printOdom = false);
    // Command Velocity Subscriber
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
    geometry_msgs::Twist getCmdVel(bool printCmdVel);
    // Laser Subscriber
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    sensor_msgs::LaserScan getLaser(bool printLaser);
    // Map Subscriber
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    nav_msgs::OccupancyGrid getMap();
    bool waitForMap();


private:
    ros::Subscriber odomSub; nav_msgs::Odometry odom;
    ros::Subscriber cmdVelSub; geometry_msgs::Twist cmd_vel;
    ros::Subscriber laserSub; sensor_msgs::LaserScan laser;
    ros::Subscriber mapSub; nav_msgs::OccupancyGrid map; 

};

}
#endif // COMMUNICATION_H
