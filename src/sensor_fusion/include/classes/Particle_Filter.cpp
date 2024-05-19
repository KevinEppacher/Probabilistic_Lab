#include "Particle_Filter.h"


ParticleFilter::ParticleFilter(int quantityParticles) : quantityParticles(quantityParticles) {}

ParticleFilter::~ParticleFilter() {}

std::vector<Particle> ParticleFilter::initializeParticles(const State &initState)
{
    std::vector<Particle> particles;
    particles.reserve(quantityParticles);

    for (int i = 0; i < quantityParticles; ++i)
    {
        // Particle particle;
        particle.state.x = initState.x;
        particle.state.y = initState.y;
        particle.state.theta = initState.theta;
        particle.weight = 1.0 / static_cast<double>(quantityParticles);
        particles.push_back(particle);

        std::cout << "Particle Init State: " << particle.state.x << ", " << particle.state.y << ", " << particle.state.theta << ", " << particle.weight << std::endl;
    }

    return particles;
}