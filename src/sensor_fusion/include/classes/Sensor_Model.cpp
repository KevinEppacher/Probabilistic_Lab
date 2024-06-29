#include "Sensor_Model.h"
#include <cmath>
#include <algorithm>

SensorModel::SensorModel(ros::NodeHandle &nodehandler) : nh(nodehandler), viz(nodehandler), subscriber(nodehandler), publisher(nodehandler)
{
    // Initialize the parameters
    nh.getParam("sensor_model/z_hit", z_hit);
    nh.getParam("sensor_model/z_short", z_short);
    nh.getParam("sensor_model/z_max", z_max);
    nh.getParam("sensor_model/z_rand", z_rand);
    nh.getParam("sensor_model/visualize_rays_percentage", visualizeRaysPercentage);
    nh.getParam("sensor_model/sigma_hit", sigma_hit);
    nh.getParam("sensor_model/lambda_short", lambda_short);

    // Initialize dynamic reconfigure server
    server = new dynamic_reconfigure::Server<sensor_fusion::SensorModelConfig>(nh.getNamespace() + "/sensor_model");

    f = boost::bind(&SensorModel::configCallback, this, _1, _2);
    
    server->setCallback(f);
}

SensorModel::~SensorModel()
{
    delete server;
}

double SensorModel::beam_range_finder_model(const sensor_msgs::LaserScan &scanMeasurement, const geometry_msgs::Pose &pose, const nav_msgs::OccupancyGrid &map)
{
    double q = 1.0;

    sensor_msgs::LaserScan z_t = scanMeasurement;

    geometry_msgs::Pose x_t = pose;

    Particle particle;

    int K = z_t.ranges.size();

    nav_msgs::Odometry odom = subscriber.getOdom();
    geometry_msgs::Pose odomPose = odom.pose.pose;

    int step = std::max(1, static_cast<int>( K * visualizeRaysPercentage / 100));

    particle.rays = rayCasting(x_t, map, z_t);

    particle.pose = pose;

    measuredRay = convertScanToRays(z_t, particle.pose);

    // for (int i = 0; i < K; i++)
    // {
    //     ROS_INFO("Ray %d", i);
    //     ROS_INFO("Particle Ray %f", particle.rays[i].length);
    //     ROS_INFO("z_t: %f", z_t.ranges[i]);
    //     ROS_INFO("Measured Ray %f", measuredRay[i].length);
    // }

    double sumError = 0, error = 0;

    z_max_range = z_t.range_max;

    for (int k = 0; k < K; k += step)
    {
        // ROS_INFO("Beam %d", k);

        double z_k = measuredRay[k].length;

        double z_star = particle.rays[k].length;

        error = z_k - z_star;

        double p = z_hit * p_hit(z_k, z_star) + z_short * p_short(z_k, z_star) + z_max * p_max(z_k, z_max_range) + z_rand * p_rand(z_k, z_max_range);

        q = q * p;

        sumError += error;
        
    }

    // ROS_INFO("Sum Error: %f", sumError);
    // publisher.publishDouble(sumError);

    // ROS_WARN("q: %f", q);

    viz.publishSimRay(particle.rays, visualizeRaysPercentage);

    viz.publishRealRay(measuredRay, visualizeRaysPercentage);


    // ROS_INFO("q: %f", q);

    return q;
}
std::vector<Ray> SensorModel::convertScanToRays(const sensor_msgs::LaserScan &z_t, const geometry_msgs::Pose &pose)
{
    std::vector<Ray> rays;
    double poseOrientation = tf::getYaw(pose.orientation);

    // Iterate over laser scan angles
    for (size_t i = 0; i < z_t.ranges.size(); ++i)
    {
        double angle = poseOrientation + z_t.angle_min + i * z_t.angle_increment;
        Ray ray;
        ray.origin = pose.position;
        ray.angle = angle;
        ray.length = z_t.ranges[i];

        rays.push_back(ray);
    }

    return rays;
}

double SensorModel::p_hit(double& z_k, double& z_star)
{
    // ROS_INFO("z_k: %f, z_max: %f", z_k, z_max_range);
    if (0 <= z_k && z_k <= z_max_range)
    {
        double normalization = numericalIntegration(z_star, z_max_range, 1000);

        double p = normalDistribution(z_k, z_star) / normalization;

        // ROS_INFO("p_hit: %f", p);
        // ROS_INFO("normalization: %f", normalization);
        // ROS_INFO("z_k: %f", z_k);
        // ROS_INFO("z_star: %f", z_star);
        // ROS_INFO(" normalDistribution(z_k, z_star): %f", normalDistribution(z_k, z_star));
        // ROS_INFO("               ");

        return p;
    }
    else
    {
        return 0;
    }
}

// Normal distribution function
double SensorModel::normalDistribution(double x, double mean)
{
    double deviation = x - mean;                                    // Calculate the deviation from the mean

    double exponent = -0.5 * pow(deviation, 2) / pow(sigma_hit, 2); // Calculate the exponent part of the normal distribution formula

    double gaussian = exp(exponent);                                // Compute the exponential term

    double normalization = 1.0 / (sqrt(2 * M_PI) * sigma_hit);      // Compute the normalization factor

    return normalization * gaussian; // Return the probability value
}

// Numerical integration using the trapezoidal rule
double SensorModel::numericalIntegration(double mean, double z_max_range, int num_steps)
{
    double step_size = z_max_range / num_steps;
    double integral = 0.0;

    for (int i = 0; i <= num_steps; ++i)
    {
        double z = i * step_size;
        double weight = (i == 0 || i == num_steps) ? 0.5 : 1.0; // Weight for trapezoidal rule
        integral += weight * normalDistribution(z, mean);
    }

    integral *= step_size;
    return integral;
}

double SensorModel::p_short(double& z_k, double& z_star)
{
    if (0 <= z_k <= z_star)
    {
        double normaliation = 1.0 / (1.0 - exp(-lambda_short * z_star));
        // ROS_INFO("normaliation: %f", normaliation);
        double p = normaliation * lambda_short * exp(-lambda_short * z_k);
        // ROS_INFO("p_short: %f", p);
        return p;
    }
    else
    {
        return 0;
    }
}



double SensorModel::p_max(double& z_k, float& z_max_range)
{
    // (condition) ? (value if_true) : (value if_false)
    return (z_k == z_max_range) ? 1.0 : 0.0;
}

double SensorModel::p_rand(double& z_k, float& z_max_range)
{
    // Probability of a random measurement
    return (0 <= z_k <= z_max_range) ? 1.0 / z_max_range : 0.0;
}

std::vector<Ray> SensorModel::rayCasting(const geometry_msgs::Pose &pose, const nav_msgs::OccupancyGrid &map, const sensor_msgs::LaserScan &z_t)
{
    // Grid parameters
    double resolution = map.info.resolution;
    int width = map.info.width;
    int height = map.info.height;

    std::vector<Ray> rays;
    Ray ray;
    ray.origin = pose.position;

    // Iterate over laser scan angles
    for (int i = 0; i < z_t.ranges.size(); i++)
    {
        // ROS_INFO(" Ray Beam %d", i);
        double particleAngle = tf::getYaw(pose.orientation);
        double angle = particleAngle + z_t.angle_min + i * z_t.angle_increment;

        ray.angle = angle;

        double x = pose.position.x;
        double y = pose.position.y;

        bool hit = false;
        double max_range = 15;
        double range = 0.0;

        while (!hit && range < max_range)
        {
            range = range + resolution;
            x = pose.position.x + range * cos(angle);
            y = pose.position.y + range * sin(angle);

            int map_x = (int)((x - map.info.origin.position.x) / resolution);
            int map_y = (int)((y - map.info.origin.position.y) / resolution);

            if (map_x >= 0 && map_x < width && map_y >= 0 && map_y < height)
            {
                int index = map_y * width + map_x;
                if (map.data[index] > 0) // If the cell is occupied
                {
                    hit = true;
                }
            }
            else
            {
                break; // Out of map bounds
            }
        }

        ray.length = range;
        rays.push_back(ray);
    }

    return rays;
}

void SensorModel::configCallback(sensor_fusion::SensorModelConfig &config, uint32_t level)
{
    z_hit = config.z_hit;
    sigma_hit = config.sigma_hit;
    z_short = config.z_short;
    lambda_short = config.lambda_short;
    z_max = config.z_max;
    z_rand = config.z_rand;
    visualizeRaysPercentage = config.visualize_rays_percentage;

    ROS_INFO("Reconfigure Request: z_hit=%f, sigma_hit=%f, z_short=%f, lambda_short=%f, z_max=%f, z_rand=%f, visualizeRaysPercentage=%f",
             z_hit, sigma_hit, z_short, lambda_short, z_max, z_rand, visualizeRaysPercentage);
}