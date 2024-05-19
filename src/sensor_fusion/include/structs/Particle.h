#ifndef PARTICLE_H
#define PARTICLE_H

#include <geometry_msgs/Pose.h>

struct Particle 
{
    geometry_msgs::Pose pose;
    double weight;
};

#endif // PARTICLE_H