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

    void Publisher::publishPose(const geometry_msgs::Pose &msg)
    {
        posePub.publish(msg);
        ROS_INFO("Particle Pose: %f, %f, %f", msg.position.x, msg.position.y, msg.orientation.z);
    }

    // Subscriber class definitions
    Subscriber::Subscriber(ros::NodeHandle &nodehandler) : Communication(nodehandler)
    {
        odomSub = nh.subscribe("odom", 10, &Subscriber::odomCallback, this);
    }

    void Subscriber::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // Handle the incoming odometry message
        // Example: ROS_INFO("Received odom: %f", msg->pose.pose.position.x);
    }

} // namespace communication