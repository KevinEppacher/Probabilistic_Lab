#include "Particle_Filter.h"

ParticleFilter::ParticleFilter(ros::NodeHandle &nodehandler, int quantityParticles) : publisher(nodehandler),
                                                                                      subscriber(nh),
                                                                                      visualizer(nh),
                                                                                      quantityParticles(quantityParticles),
                                                                                      motionModel(nh),
                                                                                      sensorModel(nh)
{
    nh.getParam("percentage_resample_random_particles", percentage_resample_random_particles);

    nh.getParam("num_particles", quantityParticles);

    server = new dynamic_reconfigure::Server<sensor_fusion::ParticleFilterConfig>(nh.getNamespace() + "/particle_filter");

    f = boost::bind(&ParticleFilter::configCallback, this, _1, _2);

    server->setCallback(f);
}

ParticleFilter::~ParticleFilter() {}

void ParticleFilter::getNodehanlder(ros::NodeHandle &nodehandler)
{
    nh = nodehandler;
}

std::vector<Particle> ParticleFilter::initializeParticles(const nav_msgs::OccupancyGrid &map)
{
    particles.reserve(quantityParticles);

    this->map = map;

    std::vector<std::pair<float, float>> free_cells = findFreeCells(map);

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

std::vector<Particle> ParticleFilter::estimatePoseWithMCL(const geometry_msgs::Twist &motionCommand, const sensor_msgs::LaserScan &sensorMeasurement, const nav_msgs::OccupancyGrid &map)
{
    geometry_msgs::PoseArray poseArrayAfterMotionModel;
    currentTime = ros::Time::now();
    double dt = (currentTime - prevTime).toSec();

    for (int i = 0; i < particles.size(); i++)
    {
        particles[i].pose = motionModel.sampleMotionModel(motionCommand, particles[i].pose, dt);

        poseArrayAfterMotionModel.poses.push_back(particles[i].pose);

        particles[i].weight = sensorModel.beam_range_finder_model(sensorMeasurement, particles[i].pose, map);
    }

    visualizer.publishPoseArrayFromMotionModel(poseArrayAfterMotionModel, false);

    particles = ParticleFilter::resampleParticles(particles);

    geometry_msgs::PoseArray resampledParticlesPoseArray = convertParticlesToPoseArray(particles);

    visualizer.publishResampledParticles(resampledParticlesPoseArray, false);

    prevTime = currentTime;

    return particles;
}

std::vector<Particle> ParticleFilter::resampleParticles(const std::vector<Particle> &particles)
{
    std::vector<Particle> resampledParticles;
    std::vector<double> weights;

    std::vector<Particle> normalizedParticles = normalizeParticles(particles, weights);

    std::string filepath = ros::package::getPath(packageName);
    Communication::CSVPlotter csvPlotter(filepath + "/measurements/Particle_Resampling_Histogramm.csv");
    csvPlotter.writeParticlesToCSV(normalizedParticles);

    std::random_device rd;
    std::mt19937 gen(rd());

    int quantityLostParticles = quantityParticles - particles.size();
    int numParticles = particles.size() + quantityLostParticles;
    int numRandomParticles = static_cast<int>(percentage_resample_random_particles * numParticles);
    int numResampledParticles = numParticles - numRandomParticles;
    std::discrete_distribution<> distribution(weights.begin(), weights.end());

    double noise_std_x = 0.05;
    double noise_std_y = 0.05;
    double noise_std_theta = 0.025;

    std::normal_distribution<double> noise_x(0, noise_std_x);
    std::normal_distribution<double> noise_y(0, noise_std_y);
    std::normal_distribution<double> noise_theta(0, noise_std_theta);

    for (int i = 0; i < numResampledParticles; i++)
    {
        int index = distribution(gen);
        Particle p = particles[index];

        p.pose.position.x += noise_x(gen);
        p.pose.position.y += noise_y(gen);
        double yaw = tf::getYaw(p.pose.orientation);
        yaw += noise_theta(gen);
        p.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        resampledParticles.push_back(p);
    }

    Particle mean_pose = calculateMeanPose(resampledParticles);

    visualizer.publishPoseWithCovariance(mean_pose, false);

    std::vector<Particle> randomParticles;
    randomParticles.reserve(numRandomParticles);

    std::vector<std::pair<float, float>> free_cells = findFreeCells(map);

    std::uniform_int_distribution<> distrib(0, free_cells.size() - 1);

    for (int i = 0; i < numRandomParticles; ++i)
    {
        int cell_index = distrib(gen);
        Particle particle;
        particle.pose.position.x = free_cells[cell_index].first;
        particle.pose.position.y = free_cells[cell_index].second;
        double randomYaw = randomOrientation(gen);
        particle.pose.orientation = tf::createQuaternionMsgFromYaw(randomYaw);
        particle.weight = 1.0 / static_cast<double>(quantityParticles);
        resampledParticles.push_back(particle);
    }

    return resampledParticles;
}

Particle ParticleFilter::calculateMeanPose(const std::vector<Particle> &particles)
{
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_yaw = 0.0;
    int numParticles = particles.size();

    for (const auto &p : particles)
    {
        sum_x += p.pose.position.x;
        sum_y += p.pose.position.y;
        sum_yaw += tf::getYaw(p.pose.orientation);
    }

    double mean_x = sum_x / numParticles;
    double mean_y = sum_y / numParticles;
    double mean_yaw = sum_yaw / numParticles;

    Particle mean;
    mean.pose.position.x = mean_x;
    mean.pose.position.y = mean_y;
    mean.pose.orientation = tf::createQuaternionMsgFromYaw(mean_yaw);

    return mean;
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

std::vector<Particle> ParticleFilter::normalizeParticles(const std::vector<Particle> &particles, std::vector<double> &weights)
{
    std::vector<Particle> normalizedParticles = particles;

    // Falls gewünscht, können Sie weights hier nicht erneut füllen, wenn weights bereits gefüllt ist
    weights.clear(); // Stellen Sie sicher, dass weights leer ist, bevor Sie es füllen

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

    for (size_t i = 0; i < weights.size(); i++)
    {
        weights[i] = weights[i] / totalWeights;
        normalizedParticles[i].weight = weights[i];
    }

    return normalizedParticles;
}

void ParticleFilter::configCallback(sensor_fusion::ParticleFilterConfig &config, uint32_t level)
{
    percentage_resample_random_particles = config.percentage_resample_random_particles;
    quantityParticles = config.num_particles;

    ROS_INFO("Reconfigure Request: percentage_resample_random_particles=%f, num_particles=%d",
             percentage_resample_random_particles, quantityParticles);
    // alpha1 = config.alpha1;
    // alpha2 = config.alpha2;
    // alpha3 = config.alpha3;
    // alpha3 = config.alpha4;
    // alpha5 = config.alpha5;
    // alpha6 = config.alpha6;

    // ROS_INFO("Reconfigure Request: alpha1=%f, alpha2=%f, alpha3=%f, alpha4=%f, alpha5=%f, alpha6=%f",
    //          alpha1, alpha2, alpha3, alpha4, alpha5, alpha6);
}