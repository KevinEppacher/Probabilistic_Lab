#include "Localization.h"

Localization::Localization()
{
    std::cout << "Lokalisierer wurde erstellt" << std::endl;

    // Initialize the subscriber
    odom_sub = nh.subscribe("odom", 1000, &Localization::odomCallback, this);
}

Localization::~Localization() 
{
    std::cout << "Lokalisierer wird zerstört" << std::endl;
}

void Localization::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Process the incoming message
    std::cout << "Received odometry data: " << std::endl;
    std::cout << "Position (x): " << msg->pose.pose.position.x << std::endl;
    std::cout << "Position (y): " << msg->pose.pose.position.y << std::endl;
    std::cout << "Orientation (z): " << msg->pose.pose.orientation.z << std::endl;
}


ParticleFilter::ParticleFilter(int quantityParticles) : quantityParticles(quantityParticles) {}

ParticleFilter::~ParticleFilter() {}

std::vector<Particle> ParticleFilter::initializeParticles(const State& initState) {
    std::vector<Particle> particles;
    particles.reserve(quantityParticles);

    for (int i = 0; i < quantityParticles; ++i) {
        Particle particle;
        particle.x = initState.x;
        particle.y = initState.y;
        particle.theta = initState.theta;
        particle.weight = 1.0 / static_cast<double>(quantityParticles);
        particles.push_back(particle);

        std::cout << "Particle Init State: " << particle.x << ", " << particle.y << ", " << particle.theta << ", " << particle.weight << std::endl;
    }

    return particles;
}




