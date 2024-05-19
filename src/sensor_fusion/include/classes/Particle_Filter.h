#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

//Include Libraries
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <random>

//Include Custom Classes
#include "Localization.h"
#include "Motion_Model.h"

//Include Structs
#include "../structs/States.h"
#include "../structs/Particle.h"


class ParticleFilter 
{
public:
    explicit ParticleFilter(int quantityParticles = 100);  // Default value for particles
    ~ParticleFilter();

    std::vector<Particle> initializeParticles(const State& initState);

    std::vector<Particle> estimatePose(const std::vector<Particle>& particles,const geometry_msgs::Twist& motionCommand, const sensor_msgs::LaserScan& z, const geometry_msgs::Pose& prevPose, const nav_msgs::OccupancyGrid& map);

private:
    Particle particle;
    int quantityParticles;
};

#endif // PARTICLE_FILTER_H
