#ifndef SENSOR_MODEL_H
#define SENSOR_MODEL_H

#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>

class Sensor_Model
{
public:
    Sensor_Model();  // constructor
    ~Sensor_Model(); // destructor

    double beam_range_finder_model(const sensor_msgs::LaserScan &z_t, const geometry_msgs::Pose &x_t, const nav_msgs::OccupancyGrid &m);

private:
    double p_hit(double z_k, double z_star, const geometry_msgs::Pose &x_t, const nav_msgs::OccupancyGrid &m);
    double p_short(double z_k, double z_star);
    double p_max(double z_k, double z_max);
    double p_rand(double z_k, double z_max);

    // Parameters for the sensor model
    double z_hit;
    double z_short;
    double z_max;
    double z_rand;
};

#endif // SENSOR_MODEL_H
