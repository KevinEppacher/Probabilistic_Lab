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

    std::vector<std::pair<float, float>> free_cells = findFreeCells(map);

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
        double randomYaw = randomOrientation(gen);
        particle.pose.orientation =  tf::createQuaternionMsgFromYaw(randomYaw); 
        particle.weight = 1.0 / static_cast<double>(quantityParticles);
        particles.push_back(particle);
        // ROS_INFO("Particle Pose: %f, %f, %f", particle.pose.position.x, particle.pose.position.y, particle.pose.orientation.z);
    }

    geometry_msgs::PoseArray particleArray = convertParticlesToPoseArray(particles);

    visualizer.publishPoseArray(particleArray, false);

    return particles;
}

std::vector<std::pair<float, float>> ParticleFilter::findFreeCells(const nav_msgs::OccupancyGrid &map)
{
    std::vector<std::pair<float, float>> free_cells;
    for (unsigned int y = 0; y < map.info.height; y++)
    {
        for (unsigned int x = 0; x < map.info.width; x++)
        {
            int index = x + y * map.info.width;
            if (map.data[index] == 0)
            { // 0 bedeutet frei
                float world_x = map.info.origin.position.x + (x + 0.5) * map.info.resolution;
                float world_y = map.info.origin.position.y + (y + 0.5) * map.info.resolution;
                free_cells.emplace_back(world_x, world_y);
            }
        }
    }
    return free_cells;
}

float ParticleFilter::randomOrientation(std::mt19937 &gen)
{
    std::uniform_real_distribution<> dis(0, 2 * M_PI);
    return dis(gen);
}

geometry_msgs::PoseArray ParticleFilter::convertParticlesToPoseArray(const std::vector<Particle> &particles)
{
    geometry_msgs::PoseArray particleArray;

    for (auto &particle : particles)
    {
        particleArray.poses.push_back(particle.pose);
    }

    return particleArray;
}

std::vector<Particle> ParticleFilter::estimatePoseWithMCL(const std::vector<Particle> &particles, const geometry_msgs::Twist &motionCommand, const sensor_msgs::LaserScan &sensorMeasurement, const nav_msgs::OccupancyGrid &map)
{
    std::vector<Particle> resampledParticles;

    double totalWeights = 0.0;

    std::vector<Particle> updatedParticles = particles;

    geometry_msgs::PoseArray poseArrayAfterMotionModel;

    for (auto &particle : updatedParticles)
    {
        // // Sample Motion Model
        geometry_msgs::Pose sampledPose = motionModel.sampleMotionModel(motionCommand, particle.pose);

        poseArrayAfterMotionModel.poses.push_back(sampledPose);

    }

    visualizer.publishPoseArrayFromMotionModel(poseArrayAfterMotionModel, true);

    // geometry_msgs::PoseArray updatedParticleArray = convertParticlesToPoseArray(updatedParticles);

    return resampledParticles;
}