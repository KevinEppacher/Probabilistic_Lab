#include "Sensor_Model.h"
#include <cmath>
#include <algorithm>

Sensor_Model::Sensor_Model()
{
    // Initialize the parameters
    z_hit = 0.8;
    z_short = 0.1;
    z_max = 0.1;
    z_rand = 0.05;
}

Sensor_Model::~Sensor_Model()
{
    // Destructor implementation
}

double Sensor_Model::beam_range_finder_model(const sensor_msgs::LaserScan &z_t, const geometry_msgs::Pose &x_t, const nav_msgs::OccupancyGrid &m)
{
    double q = 1.0;

    int K = z_t.ranges.size();
    for (int k = 0; k < K; ++k)
    {
        // Compute z_k* for the measurement z_t^k using ray casting (simplified as z_t.ranges[k] for this example)
        double z_k = z_t.ranges[k];
        double z_star = z_k; // Ray casting result should be computed here

        double p = z_hit * p_hit(z_k, z_star, x_t, m) +
                   z_short * p_short(z_k, z_star) +
                   z_max * p_max(z_k, z_t.range_max) +
                   z_rand * p_rand(z_k, z_t.range_max);

        q *= p;
    }

    return q;
}

double Sensor_Model::p_hit(double z_k, double z_star, const geometry_msgs::Pose &x_t, const nav_msgs::OccupancyGrid &m)
{
    // Probability of a hit
    double sigma_hit = 0.2; // Standard deviation of the measurement noise
    double denom = sqrt(2 * M_PI * sigma_hit * sigma_hit);
    double expo = exp(-0.5 * pow((z_k - z_star) / sigma_hit, 2));
    return expo / denom;
}

double Sensor_Model::p_short(double z_k, double z_star)
{
    // Probability of a short reading
    if (z_k < 0 || z_k > z_star)
        return 0.0;
    double lambda_short = 0.1;
    return lambda_short * exp(-lambda_short * z_k);
}

double Sensor_Model::p_max(double z_k, double z_max)
{
    // Probability of a max range reading
    return z_k == z_max ? 1.0 : 0.0;
}

double Sensor_Model::p_rand(double z_k, double z_max)
{
    // Probability of a random measurement
    return (z_k >= 0 && z_k <= z_max) ? 1.0 / z_max : 0.0;
}
