#include "Particle_Filter.h"


// ParticleFilter::ParticleFilter(int quantityParticles) : quantityParticles(quantityParticles) {}

// ParticleFilter::~ParticleFilter() {}

// std::vector<Particle> ParticleFilter::initializeParticles(const State &initState)
// {
//     std::vector<Particle> particles;
//     particles.reserve(quantityParticles);

//     for (int i = 0; i < quantityParticles; ++i)
//     {
//         Particle particle;
//         particle.x = initState.x;
//         particle.y = initState.y;
//         particle.theta = initState.theta;
//         particle.weight = 1.0 / static_cast<double>(quantityParticles);
//         particles.push_back(particle);

//         std::cout << "Particle Init State: " << particle.x << ", " << particle.y << ", " << particle.theta << ", " << particle.weight << std::endl;
//     }

//     return particles;
// }