#ifndef PARTICLE_H
#define PARTICLE_H

#include <geometry_msgs/Pose.h>
#include <vector>
#include "Ray.h"

struct Particle 
{
    geometry_msgs::Pose pose;
    double weight;
    std::vector<Ray> rays;
};

#endif // PARTICLE_H