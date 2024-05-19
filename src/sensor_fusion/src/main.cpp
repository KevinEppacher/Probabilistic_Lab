#include "ros/ros.h"
//Include Custom Classes
#include "Localization.h"
#include "Particle_Filter.h"
#include "Motion_Model.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    ParticleFilter particleFilter(100);
    State robotState(0, 0, 0);

    std::vector<Particle> particles = particleFilter.initializeParticles(robotState);

    while (ros::ok()) 
    {
        geometry_msgs::Twist motionCommand;
        sensor_msgs::LaserScan sensorMeasurement;
        geometry_msgs::Pose prevPose;
        nav_msgs::OccupancyGrid map;
        particleFilter.estimatePose(particles, motionCommand, sensorMeasurement, prevPose, map);


        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
