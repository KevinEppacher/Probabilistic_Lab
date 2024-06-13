#include "ros/ros.h"
// Include Custom Classes
#include "Particle_Filter.h"
#include "Motion_Model.h"
#include "Communication.h"
#include "Sensor_Model.h"
#include "Visualizer.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle nh;

    double loop_frequency;
    int numParticles;
    nh.getParam("loop_rate", loop_frequency);
    nh.getParam("num_particles", numParticles);

    ros::Rate loop_rate(loop_frequency);

    // Setup
    State robotState(0, 0, 0);

    Communication::Subscriber subscriber(nh);

    nav_msgs::OccupancyGrid map = subscriber.getMap();

    ParticleFilter particleFilter(nh, numParticles);

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

    Visualizer::Visualizer visualizer(nh);
    visualizer.clearMarkers();

    return 0;
}
