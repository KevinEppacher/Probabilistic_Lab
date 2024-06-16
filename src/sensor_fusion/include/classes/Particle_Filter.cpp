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

std::vector<Particle> ParticleFilter::initializeParticles(const nav_msgs::OccupancyGrid &map)
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
    std::vector<Particle> updatedParticles;

    geometry_msgs::PoseArray poseArrayAfterMotionModel;

    this->map = map;

    double weight;

    for (auto &particle : particles)
    {
        geometry_msgs::Pose sampledPose = motionModel.sampleMotionModel(motionCommand, particle.pose);

        weight = sensorModel.beam_range_finder_model(sensorMeasurement, sampledPose, map);

        // ROS_INFO("Weight: %f", weight);

        Particle updatedParticle = particle;

        updatedParticle.pose = sampledPose;

        updatedParticle.weight = weight;

        updatedParticles.push_back(updatedParticle);

        poseArrayAfterMotionModel.poses.push_back(sampledPose);

        // if (isPoseInFreeCell(sampledPose, map))
        // {
        //     Particle updatedParticle = particle;

        //     updatedParticle.pose = sampledPose;

        //     updatedParticle.weight = weight;

        //     updatedParticles.push_back(updatedParticle);

        //     poseArrayAfterMotionModel.poses.push_back(sampledPose);
        // }
    }

    visualizer.publishPoseArrayFromMotionModel(poseArrayAfterMotionModel, false);

    // for (auto &particle : updatedParticles)
    // {
    //     ROS_INFO("Particle Pose: %f, %f, %f, Particle Weight %f", particle.pose.position.x, particle.pose.position.y, particle.pose.orientation.z, particle.weight);
    // }

    // Resample particles based on their weights
    std::vector<Particle> resampleParticles = ParticleFilter::resampleParticles(updatedParticles);

    // for(auto& particle : resampleParticles)
    // {
    //     ROS_INFO("Resampled Particle Pose: %f, %f, %f, Particle Weight %f", particle.pose.position.x, particle.pose.position.y, particle.pose.orientation.z, particle.weight);
    // }

    geometry_msgs::PoseArray resampledParticlesPoseArray = convertParticlesToPoseArray(resampleParticles);

    visualizer.publishResampledParticles(resampledParticlesPoseArray, false);

    return resampleParticles;
}

std::vector<Particle> ParticleFilter::resampleParticles(std::vector<Particle> &particles)
{
    std::vector<Particle> resampledParticles;
    std::vector<double> weights;
    double totalWeights = 0.0, newWeight = 0.0;

    for(auto& particle : particles)
    {
        weights.push_back(particle.weight);
        totalWeights += particle.weight;
    }

    // Normiere die Gewichte
    for(int i = 0; i < particles.size(); i++)
    {
        particles[i].weight /= totalWeights;
        newWeight += particles[i].weight;
        // ROS_INFO("Particle Weight: %f", particles[i].weight);
    }

    if(newWeight < 0.99 || newWeight > 1.01 )
    {
        ROS_WARN("Sum of Particles Weights: %f", newWeight);
    }

    // Normiere die Gewichte in der weights-Liste
    for(auto& weight : weights)
    {
        weight /= totalWeights;
    }

    std::random_device rd;
    std::mt19937 gen(rd());

    double quantityLostParticles = quantityParticles - particles.size();

    ROS_INFO("Quantity Lost Particles: %f", quantityLostParticles);

    int numParticles = particles.size() + quantityLostParticles;
    int numRandomParticles = static_cast<int>(0.2 * numParticles); // 10% der Partikel zufällig verteilen
    int numResampledParticles = numParticles - numRandomParticles;

    // Resample existing particles
    for(int i = 0; i < numResampledParticles; i++)
    {
        int index = std::discrete_distribution<int>(weights.begin(), weights.end())(gen);
        Particle resampledParticle = particles[index];
        resampledParticle.weight = 1.0 / numParticles;
        resampledParticles.push_back(resampledParticle);
        ROS_INFO("Resampled Particle Pose: %f, %f, %f", resampledParticle.pose.position.x, resampledParticle.pose.position.y, resampledParticle.pose.orientation.z);
    }


    std::vector<Particle> randomParticles;
    randomParticles.reserve(numRandomParticles);

    std::vector<std::pair<float, float>> free_cells = findFreeCells(map);

    // Erzeugen von Partikeln an zufälligen freien Stellen
    std::uniform_int_distribution<> distrib(0, free_cells.size() - 1);

    for (int i = 0; i < numRandomParticles; ++i)
    {
        int cell_index = distrib(gen); // Zufällige Zellenindex
        Particle particle;
        particle.pose.position.x = free_cells[cell_index].first;
        particle.pose.position.y = free_cells[cell_index].second;
        double randomYaw = randomOrientation(gen);
        particle.pose.orientation = tf::createQuaternionMsgFromYaw(randomYaw);
        particle.weight = 1.0 / static_cast<double>(quantityParticles);
        resampledParticles.push_back(particle);
        // ROS_INFO("Particle Pose: %f, %f, %f", particle.pose.position.x, particle.pose.position.y, particle.pose.orientation.z);
    }

    return resampledParticles;
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