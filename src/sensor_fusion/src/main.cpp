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
    double loop_duration = 1.0 / loop_frequency; // Dauer eines Schleifendurchlaufs in Sekunden

    Communication::Subscriber subscriber(nh);

    nav_msgs::OccupancyGrid map = subscriber.getMap();

    ParticleFilter particleFilter(nh, numParticles);

    std::vector<Particle> particles = particleFilter.initializeParticles(map);

    while (ros::ok())
    {
        ros::Time start_time = ros::Time::now(); // Beginn des Schleifendurchlaufs

        geometry_msgs::Twist motionCommand = subscriber.getCmdVel(false);
        sensor_msgs::LaserScan laserMeasurement = subscriber.getLaser(false);

        std::vector<Particle> updatedParticles = particleFilter.estimatePoseWithMCL(particles, motionCommand, laserMeasurement, map);

        ROS_INFO("Updated Particles: %lu", updatedParticles.size());

        ros::Time end_time = ros::Time::now(); // Ende des Schleifendurchlaufs
        ros::Duration elapsed_time = end_time - start_time; // Dauer des Schleifendurchlaufs

        // Überprüfen, ob die Schleifenrate eingehalten wird
        if (elapsed_time.toSec() > loop_duration)
        {
            ROS_WARN("Loop rate missed! Elapsed time: %f seconds, Loop duration: %f seconds", elapsed_time.toSec(), loop_duration);
        }
        else
        {
            ROS_INFO("Loop rate maintained. Elapsed time: %f seconds", elapsed_time.toSec());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    Visualizer::Visualizer visualizer(nh);
    visualizer.clearMarkers();

    return 0;
}
