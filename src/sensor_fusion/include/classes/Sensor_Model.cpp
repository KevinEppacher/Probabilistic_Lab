#include "Sensor_Model.h"
#include <cmath>
#include <algorithm>

SensorModel::SensorModel(ros::NodeHandle &nodehandler) : nh(nodehandler), viz(nodehandler), subscriber(nodehandler)
{
    // Initialize the parameters
    nh.getParam("z_hit", z_hit);
    nh.getParam("z_short", z_short);
    nh.getParam("z_max", z_max);
    nh.getParam("z_rand", z_rand);
    nh.getParam("visualize_rays_percentage", visualizeRaysPercentage);
}

SensorModel::~SensorModel()
{
    // Destructor implementation
}

double SensorModel::beam_range_finder_model(const sensor_msgs::LaserScan &z_t, const geometry_msgs::Pose &x_t, const nav_msgs::OccupancyGrid &m)
{
    double q = 1.0;

    Particle particle;

    int K = z_t.ranges.size();

    std::vector<Ray> measuredRay;
    ;

    nav_msgs::Odometry odom = subscriber.getOdom();
    geometry_msgs::Pose odomPose = odom.pose.pose;

    for (int k = 0; k < K; ++k)
    {
        particle.rays = rayCasting(x_t, m, z_t);

        measuredRay = convertScanToRays(z_t, odomPose);

        double z_k = measuredRay[k].length;

        double z_star = particle.rays[k].length;

        double p = z_hit * p_hit(z_k, z_star, x_t, m) + z_short * p_short(z_k, z_star) + z_max * p_max(z_k, z_t.range_max) + z_rand * p_rand(z_k, z_t.range_max);

        q *= p;
    }

    viz.publishSimRay(particle.rays, visualizeRaysPercentage);

    viz.publishRealRay(measuredRay, visualizeRaysPercentage);

    ROS_INFO("q: %f", q);

    return q;
}

std::vector<Ray> SensorModel::convertScanToRays(const sensor_msgs::LaserScan &z_t, const geometry_msgs::Pose &pose)
{
    std::vector<Ray> rays;

    // Iterate over laser scan angles
    for (size_t i = 0; i < z_t.ranges.size(); ++i)
    {
        double angle = z_t.angle_min + i * z_t.angle_increment;

        Ray ray;
        ray.origin = pose.position;
        ray.angle = angle;
        ray.length = z_t.ranges[i];

        rays.push_back(ray);
    }

    return rays;
}

double SensorModel::p_hit(double z_k, double z_star, const geometry_msgs::Pose &x_t, const nav_msgs::OccupancyGrid &m)
{

    if (0 <= z_k <= z_max)
    {
        /* code */
    }
    else
    {
        return 0;
    }

    // Probability of a hit
    double sigma_hit = 0.2; // Standard deviation of the measurement noise
    double denom = sqrt(2 * M_PI * sigma_hit * sigma_hit);
    double expo = exp(-0.5 * pow((z_k - z_star) / sigma_hit, 2));
    return expo / denom;
}

double SensorModel::p_short(double z_k, double z_star)
{
    // Probability of a short reading
    if (z_k < 0 || z_k > z_star)
        return 0.0;
    double lambda_short = 0.1;
    return lambda_short * exp(-lambda_short * z_k);
}

double SensorModel::p_max(double z_k, double z_max)
{
    // Probability of a max range reading
    return z_k == z_max ? 1.0 : 0.0;
}

double SensorModel::p_rand(double z_k, double z_max)
{
    // Probability of a random measurement
    return (z_k >= 0 && z_k <= z_max) ? 1.0 / z_max : 0.0;
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
        double angle = z_t.angle_min + i * z_t.angle_increment;

        ray.angle = angle;

        double x = pose.position.x;
        double y = pose.position.y;

        bool hit = false;
        double max_range = z_t.range_max;
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

// Normalverteilungsfunktion
double normalDistribution(double x, double mean)
{
    double variance = 0.2;
    double exponent = exp(-0.5 * pow((x - mean), 2) / pow(variance, 2));
    return (1.0 / (sqrt(2 * M_PI) * variance)) * exponent;
}