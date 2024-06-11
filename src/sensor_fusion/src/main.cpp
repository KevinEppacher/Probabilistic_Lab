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

    nav_msgs::OccupancyGrid map = subscriber.getMap();

    ParticleFilter particleFilter(nh, 10);

    std::vector<Particle> particles = particleFilter.initializeParticles(robotState, map);

    while (ros::ok())
    {
        geometry_msgs::Twist motionCommand = subscriber.getCmdVel(false);
        sensor_msgs::LaserScan laserMeasurement = subscriber.getLaser(false);

        std::vector<Particle> updatedParticles = particleFilter.estimatePoseWithMCL(particles, motionCommand, laserMeasurement, map);

        ROS_INFO("Updated Particles: %lu", updatedParticles.size());

        particles = updatedParticles;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
