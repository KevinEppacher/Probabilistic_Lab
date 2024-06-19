#include "Particle_Filter.h"

ParticleFilter::ParticleFilter(ros::NodeHandle &nodehandler, int quantityParticles) : publisher(nodehandler),
                                                                                      subscriber(nh),
                                                                                      visualizer(nh),
                                                                                      quantityParticles(quantityParticles),
                                                                                      motionModel(nh),
                                                                                      sensorModel(nh)
{
    nh.getParam("percentage_resample_random_particles", percentage_resample_random_particles);
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

    this->map = map;

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

std::vector<Particle> ParticleFilter::estimatePoseWithMCL(std::vector<Particle> &particles, const geometry_msgs::Twist &motionCommand, const sensor_msgs::LaserScan &sensorMeasurement, const nav_msgs::OccupancyGrid &map)
{
    std::vector<Particle> updatedParticles;

    geometry_msgs::PoseArray poseArrayAfterMotionModel;

    for (auto &particle : particles)
    {
        geometry_msgs::Pose sampledPose = motionModel.sampleMotionModel(motionCommand, particle.pose);

        double weight = sensorModel.beam_range_finder_model(sensorMeasurement, sampledPose, map);

        if (isPoseInFreeCell(sampledPose, map))
        {
            Particle updatedParticle = particle;

            updatedParticle.pose = sampledPose;

            updatedParticle.weight = weight;

            updatedParticles.push_back(updatedParticle);

            poseArrayAfterMotionModel.poses.push_back(sampledPose);
        }
    }

    visualizer.publishPoseArrayFromMotionModel(poseArrayAfterMotionModel, false);

    // Resample particles based on their weights
    std::vector<Particle> resampledParticles = ParticleFilter::resampleParticles(updatedParticles);

    geometry_msgs::PoseArray resampledParticlesPoseArray = convertParticlesToPoseArray(resampledParticles);

    visualizer.publishResampledParticles(resampledParticlesPoseArray, false);

    particles = resampledParticles;

    return resampledParticles;
}

std::vector<Particle> ParticleFilter::resampleParticles(const std::vector<Particle> &particles)
{
    std::vector<Particle> resampledParticles;
    std::vector<double> weights;

    for (const auto &particle : particles)
    {
        weights.push_back(particle.weight);
    }

    double totalWeights = std::accumulate(weights.begin(), weights.end(), 0.0);


    if (totalWeights == 0)
    {
        ROS_WARN("Total particle weights sum to zero.");
        return particles;
    }

    for(int i = 0; i < weights.size(); i++)
    {
        weights[i] = weights[i] / totalWeights;
    }

    // for(auto &weight : weights)
    // {
    //     ROS_INFO("Weights: %f", weight);
    // }

    // printHistogram(particles, 20);

    std::random_device rd;
    std::mt19937 gen(rd());

    int quantityLostParticles = quantityParticles - particles.size();
    int numParticles = particles.size() + quantityLostParticles;
    int numRandomParticles = static_cast<int>(percentage_resample_random_particles * numParticles);
    int numResampledParticles = numParticles - numRandomParticles;
    std::discrete_distribution<> distribution(weights.begin(), weights.end());
    
    // Resample existing particles
    for (int i = 0; i < numResampledParticles; i++)
    {
        int index = distribution(gen);
        resampledParticles.push_back(particles[index]);
        // ROS_INFO("Resampled Particle x: %f, y: %f, z: %f", resampledParticles[i].pose.position.x, resampledParticles[i].pose.position.y, resampledParticles[i].pose.orientation.z);
    }


    // Resample random particles

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

void ParticleFilter::printHistogram(const std::vector<Particle> &particles, int numBins)
{
    std::vector<int> histogram(numBins, 0);
    double maxWeight = 0.0;

    // Find the maximum weight
    for (const auto &particle : particles)
    {
        if (particle.weight > maxWeight)
        {
            maxWeight = particle.weight;
        }
    }

    // ROS_INFO("Max Weight: %f", maxWeight);

    // Bin the weights
    for (const auto &particle : particles)
    {
        int binIndex = std::min(static_cast<int>(std::floor((particle.weight / maxWeight) * numBins)), numBins - 1);
        histogram[binIndex]++;
    }

    // Print the histogram
    std::cout << "Particle Weights Histogram:" << std::endl;
    for (int i = 0; i < numBins; i++)
    {
        std::cout << std::setw(4) << i << " | ";
        int numStars = histogram[i];
        for (int j = 0; j < numStars; j++)
        {
            std::cout << "*";
        }
        std::cout << std::endl;
    }
}

void ParticleFilter::printWeights(const std::vector<Particle> &particles)
{
    
}   