#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "Localization.h"
#include "Motion_Model.h"

// struct Particle 
// {
//     double x, y, theta;  // Particle state variables
//     double weight;       // Weight of this particle
// };

// class ParticleFilter 
// {
// public:
//     explicit ParticleFilter(int quantityParticles = 100);  // Default value for particles
//     ~ParticleFilter();

//     std::vector<Particle> initializeParticles(const State& initState);

// private:
//     int quantityParticles;
// };

#endif // PARTICLE_FILTER_H
