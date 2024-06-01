#include "Communication.h"

namespace Communication
{

    // Communication class definitions
    Communication::Communication(ros::NodeHandle &nodehandler) : nh(nodehandler) {}

    Communication::~Communication() {}

    // Publisher class definitions
    Publisher::Publisher(ros::NodeHandle &nodehandler) : Communication(nodehandler)
    {
        twistPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        posePub = nh.advertise<geometry_msgs::Pose>("pose", 10);
    }

    void Publisher::publishTwist(const geometry_msgs::Twist &msg)
    {
        twistPub.publish(msg);
    }

    void Publisher::publishPose(const geometry_msgs::Pose &msg, bool printPose)
    {
        posePub.publish(msg);
        if (printPose)
            ROS_INFO("Particle Pose: %f, %f, %f", msg.position.x, msg.position.y, msg.orientation.z);
    }

    // Subscriber class definitions
    Subscriber::Subscriber(ros::NodeHandle &nodehandler) : Communication(nodehandler)
    {
        odomSub = nh.subscribe("odom", 10, &Subscriber::odomCallback, this);
        cmdVelSub = nh.subscribe("cmd_vel", 10, &Subscriber::cmdVelCallback, this);
        laserSub = nh.subscribe("scan", 10, &Subscriber::laserCallback, this);
        mapSub = nh.subscribe("map", 10, &Subscriber::mapCallback, this);
    }

    void Subscriber::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // Handle the incoming odometry message
        odom = *msg;
    }

    

    nav_msgs::Odometry Subscriber::getOdom(bool printOdom)
    {
        if (printOdom)
        {   
            ROS_INFO("Received odom: %f", odom.pose.pose.position.x);
            ROS_INFO("Received odom: %f", odom.pose.pose.position.y);
            ROS_INFO("Received odom: %f", odom.pose.pose.orientation.z);
        }
        return odom;
    }

    void Subscriber::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        // Handle the incoming odometry message
        cmd_vel = *msg;
    }

    geometry_msgs::Twist Subscriber::getCmdVel(bool printCmdVel)
    {
        if (printCmdVel)
        {
            ROS_INFO("Received cmd_vel linear x: %f", cmd_vel.linear.x);
            ROS_INFO("Received cmd_vel angular z: %f", cmd_vel.angular.z);
        }
        return cmd_vel;
    }

    void Subscriber::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        // Handle the incoming odometry message
        laser = *msg;
    }

    sensor_msgs::LaserScan Subscriber::getLaser(bool printLaser)
    {
        if (laser.ranges.empty()) {
            ROS_WARN("Laser data is empty.");
            return laser;
        }
        if (printLaser) {
            ROS_INFO("Received laser: %f", laser.ranges[0]);
        }
        return laser;
    }

    void Subscriber::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        // Handle the incoming odometry message
        map = *msg;
    }

    nav_msgs::OccupancyGrid Subscriber::getMap(bool printMap)
    {
        if (printMap)
        {
            ROS_INFO("Received map width: %u", map.info.width);
            ROS_INFO("Received map height: %u", map.info.height);
        }
        return map;
    }


} // namespace communication
