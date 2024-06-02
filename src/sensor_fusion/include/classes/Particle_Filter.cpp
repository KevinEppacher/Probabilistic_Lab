#include "Particle_Filter.h"

ParticleFilter::ParticleFilter(ros::NodeHandle &nodehandler, int quantityParticles) : publisher(nodehandler),
                                                                                      subscriber(nh),
                                                                                      visualizer(nh),
                                                                                      quantityParticles(quantityParticles),
                                                                                      motionModel(nh)
{
    // Communication::Publisher publisher(nh);
}

ParticleFilter::~ParticleFilter() {}

void ParticleFilter::getNodehanlder(ros::NodeHandle &nodehandler)
{
    nh = nodehandler;
}

std::vector<Particle> ParticleFilter::initializeParticles(const State &initState, const nav_msgs::OccupancyGrid &map)
{
    std::vector<Particle> particles;
    particles.reserve(quantityParticles);

    std::vector<std::pair<float, float>> free_cells;

    // Identifizieren freier Zellen
    for (unsigned int y = 0; y < map.info.height; y++)
    {
        for (unsigned int x = 0; x < map.info.width; x++)
        {
            int index = x + y * map.info.width;
            if (map.data[index] == 0)
            { // 0 bedeutet frei
                // Umrechnung von Zellenkoordinaten in Weltkoordinaten
                float world_x = map.info.origin.position.x + (x + 0.5) * map.info.resolution;
                float world_y = map.info.origin.position.y + (y + 0.5) * map.info.resolution;
                free_cells.emplace_back(world_x, world_y);
            }
        }
    }

    // Erzeugen von Partikeln an zufälligen freien Stellen
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(0, free_cells.size() - 1);

    for (int i = 0; i < quantityParticles; ++i)
    {
        int cell_index = distrib(gen); // Zufällige Zellenindex
        Particle particle;
        particle.pose.position.x = free_cells[cell_index].first;
        particle.pose.position.y = free_cells[cell_index].second;
        particle.pose.orientation.z = initState.theta; // Initialwinkel beibehalten oder ebenfalls zufällig wählen
        particle.weight = 1.0 / static_cast<double>(quantityParticles);
        particles.push_back(particle);
    }

    return particles;
}

std::vector<Particle> ParticleFilter::estimatePoseWithMCL(const std::vector<Particle> &particles, const geometry_msgs::Twist &motionCommand, const sensor_msgs::LaserScan &sensorMeasurement, const geometry_msgs::Pose &currentPose, const nav_msgs::OccupancyGrid &map)
{
    std::vector<Particle> resampledParticles;

    double totalWeights = 0.0;

    std::vector<Particle> updatedParticles = particles;

    for (auto &particle : updatedParticles)
    {
        // // Sample Motion Model
        geometry_msgs::Twist sampledMotion = motionModel.sampleMotionModel(motionCommand, currentPose);

        // ROS_INFO("Sampled Motion: %f, %f, %f", sampledMotion.linear.x, sampledMotion.linear.y, sampledMotion.angular.z);
    }

    return resampledParticles;
}