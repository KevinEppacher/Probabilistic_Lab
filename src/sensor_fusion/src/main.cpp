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
    geometry_msgs::Twist motionCommand;
    sensor_msgs::LaserScan sensorMeasurement;
    geometry_msgs::Pose prevPose;
    nav_msgs::OccupancyGrid map;

    ParticleFilter particleFilter(nh, 100);
    std::vector<Particle> particles = particleFilter.initializeParticles(robotState);

    while (ros::ok())
    {
        particleFilter.estimatePoseWithMCL(particles, motionCommand, sensorMeasurement, prevPose, map);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
