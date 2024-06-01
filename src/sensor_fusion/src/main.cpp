#include "ros/ros.h"
// Include Custom Classes
#include "Particle_Filter.h"
#include "Motion_Model.h"
#include "Communication.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle nh;

    ros::Rate loop_rate(1);

    // Setup
    State robotState(0, 0, 0);

    Communication::Subscriber subscriber(nh);

    nav_msgs::OccupancyGrid map = subscriber.getMap(true);
    ParticleFilter particleFilter(nh, 100);
    std::vector<Particle> particles = particleFilter.initializeParticles(robotState);

    while (ros::ok())
    {      
        geometry_msgs::Twist motionCommand = subscriber.getCmdVel(false);
        nav_msgs::Odometry odom = subscriber.getOdom(false);
        geometry_msgs::Pose currentPose = odom.pose.pose;
        sensor_msgs::LaserScan laserMeasurement = subscriber.getLaser(false);

        particleFilter.estimatePoseWithMCL(particles, motionCommand, laserMeasurement, currentPose, map);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
