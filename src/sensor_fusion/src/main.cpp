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

    Communication::Subscriber subscriber(nh);

    nav_msgs::OccupancyGrid map = subscriber.getMap();

    ParticleFilter particleFilter(nh, 100);

    std::vector<Particle> particles = particleFilter.initializeParticles(map);

    while (ros::ok())
    {
        geometry_msgs::Twist motionCommand = subscriber.getCmdVel(false);
        
        sensor_msgs::LaserScan laserMeasurement = subscriber.getLaser(false);

        std::vector<Particle> updatedParticles = particleFilter.estimatePoseWithMCL(motionCommand, laserMeasurement, map);

        ros::spinOnce();
    }

    Visualizer::Visualizer visualizer(nh);
    visualizer.clearMarkers();

    return 0;
}
