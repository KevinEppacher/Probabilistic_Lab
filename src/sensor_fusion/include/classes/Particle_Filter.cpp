#include "Particle_Filter.h"

ParticleFilter::ParticleFilter(ros::NodeHandle &nodehandler, int quantityParticles) : publisher(nodehandler),
                                                                                      subscriber(nh),
                                                                                      visualizer(nh),
                                                                                      quantityParticles(quantityParticles),
                                                                                      motionModel(nh),
                                                                                      sensorModel(nh)
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
        particle.pose.orientation = tf::createQuaternionMsgFromYaw(randomYaw);
        particle.weight = 1.0 / static_cast<double>(quantityParticles);
        particles.push_back(particle);
        // ROS_INFO("Particle Pose: %f, %f, %f", particle.pose.position.x, particle.pose.position.y, particle.pose.orientation.z);
    }

    geometry_msgs::PoseArray particleArray = convertParticlesToPoseArray(particles);

    visualizer.publishInitialParticles(particleArray, false);

    return particles;
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
    double totalWeights = 0.0;

    std::vector<Particle> updatedParticles;

    geometry_msgs::PoseArray poseArrayAfterMotionModel;

    // SensorModel sensorModel;

    double weight;

    for (auto &particle : particles)
    {
        geometry_msgs::Pose sampledPose = motionModel.sampleMotionModel(motionCommand, particle.pose);

        weight = sensorModel.beam_range_finder_model(sensorMeasurement, sampledPose, map);

        if (isPoseInFreeCell(sampledPose, map))
        {
            Particle updatedParticle = particle;

            updatedParticle.pose = sampledPose;

            updatedParticle.weight = weight;

            updatedParticles.push_back(updatedParticle);

            totalWeights += weight;

            poseArrayAfterMotionModel.poses.push_back(sampledPose);
        }
    }

    visualizer.publishPoseArrayFromMotionModel(poseArrayAfterMotionModel, false);

    // for (auto &particle : updatedParticles)
    // {
    //     particle.weight /= totalWeights;
    // }

    for(int i = 0; i < updatedParticles.size(); i++)
    {
        updatedParticles[i].weight /= totalWeights;
    }

    for (auto &particle : updatedParticles)
    {
        ROS_INFO("Particle Pose: %f, %f, %f, Particle Weight %f", particle.pose.position.x, particle.pose.position.y, particle.pose.orientation.z, particle.weight);
    }

    // Resample particles based on their weights
    std::vector<Particle> resampleParticles = ParticleFilter::resampleParticles(updatedParticles);

    for(auto& particle : resampleParticles)
    {
        // ROS_INFO("Resampled Particle Pose: %f, %f, %f, Particle Weight %f", particle.pose.position.x, particle.pose.position.y, particle.pose.orientation.z, particle.weight);
    }

    return updatedParticles;
}

bool ParticleFilter::isPoseInFreeCell(const geometry_msgs::Pose &pose, const nav_msgs::OccupancyGrid &map)
{
    int x = (pose.position.x - map.info.origin.position.x) / map.info.resolution;
    int y = (pose.position.y - map.info.origin.position.y) / map.info.resolution;

    if (x >= 0 && x < map.info.width && y >= 0 && y < map.info.height)
    {
        int index = x + y * map.info.width;
        return map.data[index] == 0; // 0 bedeutet frei
    }

    return false;
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

std::vector<Particle> ParticleFilter::resampleParticles(const std::vector<Particle> &particles)
{
    std::vector<Particle> resampledParticles;
    resampledParticles.reserve(particles.size());

    // Compute the cumulative weights
    std::vector<double> cumulativeWeights(particles.size(), 0.0);
    cumulativeWeights[0] = particles[0].weight;
    for (size_t i = 1; i < particles.size(); ++i)
    {
        cumulativeWeights[i] = cumulativeWeights[i - 1] + particles[i].weight;
    }

    // Generate a random start point
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0 / particles.size());
    double start = dis(gen);

    // Resample the particles
    size_t index = 0;
    for (size_t i = 0; i < particles.size(); ++i)
    {
        double target = start + i * (1.0 / particles.size());
        while (target > cumulativeWeights[index])
        {
            index++;
        }
        resampledParticles.push_back(particles[index]);

        // ROS_INFO(" Resampled Particle Pose: %f, %f, %f, Particle Weight %f", particles[index].pose.position.x, particles[index].pose.position.y, particles[index].pose.orientation.z, particles[index].weight);
    }

    return resampledParticles;
}
