#include "ros/ros.h"
#include "Localization.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle nh;

    Localization localization;
    ros::Rate loop_rate(10);

    ParticleFilter particleFilter(100); // Now providing a default quantity of particles
    State robotState(0, 0, 0);

    auto particles = particleFilter.initializeParticles(robotState);

    while (ros::ok()) 
    {




        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
