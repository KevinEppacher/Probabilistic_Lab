#include "Particle_Filter.h"

ParticleFilter::ParticleFilter(ros::NodeHandle &nodehandler, int quantityParticles) 
    : publisher(nodehandler),subscriber(nh), quantityParticles(quantityParticles)
{
    // Communication::Publisher publisher(nh);
}

ParticleFilter::~ParticleFilter() {}

void ParticleFilter::getNodehanlder(ros::NodeHandle &nodehandler)
{
    nh = nodehandler;
}

std::vector<Particle> ParticleFilter::initializeParticles(const State &initState)
{
    std::vector<Particle> particles;
    particles.reserve(quantityParticles);

    for (int i = 0; i < quantityParticles; ++i)
    {
        // Particle particle;
        particle.pose.position.x = initState.x;
        particle.pose.position.y = initState.y;
        particle.pose.orientation.z = initState.theta;
        particle.weight = 1.0 / static_cast<double>(quantityParticles);
        particles.push_back(particle);

        std::cout << "Particle Init State: " << particle.pose.position.x << ", " << particle.pose.position.y << ", " << particle.pose.orientation.z << ", " << particle.weight << std::endl;
    }

    return particles;
}
std::vector<Particle> ParticleFilter::estimatePoseWithMCL(const std::vector<Particle> &particles,
                                                   const geometry_msgs::Twist &motionCommand,
                                                   const sensor_msgs::LaserScan &sensorMeasurement,
                                                   const geometry_msgs::Pose &prevPose,
                                                   const nav_msgs::OccupancyGrid &map)
{
    std::vector<Particle> resampledParticles;

    double totalWeights = 0.0;

    std::vector<Particle> updatedParticles = particles;

    // Communication::Publisher publisher(nh);

    for (auto &particle : updatedParticles)
    {
        // // Sample Motion Model
        // MotionModel motionModel;
        // geometry_msgs::Twist sampledMotion = motionModel.sampleMotionModel();

        publisher.publishPose(particle.pose, false);
        nav_msgs::Odometry odom = subscriber.getOdom(false);

        // ROS_INFO("Particle Pose: %f, %f, %f", particle.pose.position.x, particle.pose.position.y, particle.pose.orientation.z);
    }

    return resampledParticles;
}