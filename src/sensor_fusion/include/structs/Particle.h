#ifndef PARTICLE_H
#define PARTICLE_H

#include "States.h"

struct Particle 
{
    State state;
    double weight;       // Weight of this particle
};

#endif // PARTICLE_H