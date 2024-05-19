#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H
//Include Libraries
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
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

private:
    Particle particle;
    int quantityParticles;
};

#endif // PARTICLE_FILTER_H
