#ifndef SensorModel_H
#define SensorModel_H

#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

// Dynamic Reconfigure
#include <dynamic_reconfigure/server.h>
#include <mutex>

// Dynamic Reconfigure Sensor Model Parameters
#include <sensor_fusion/SensorModelConfig.h>

// Custom classes
#include "Visualizer.h"
#include "Communication.h"

class SensorModel
{
public:
    SensorModel(ros::NodeHandle& nodehandler);  // constructor
    SensorModel(){};
    ~SensorModel(); // destructor

    double beam_range_finder_model(const sensor_msgs::LaserScan &scanMeasurement, const geometry_msgs::Pose &pose, const nav_msgs::OccupancyGrid &map);

private:
    double p_hit(double& z_k, double& z_star);
    double p_short(double& z_k, double& z_star);
    double p_max(double& z_k, float& z_max_range);
    double p_rand(double& z_k, float& z_max_range);
    std::vector<Ray> rayCasting(const geometry_msgs::Pose &pose, const nav_msgs::OccupancyGrid &map, const sensor_msgs::LaserScan &z_t);
    std::vector<Ray> convertScanToRays(const sensor_msgs::LaserScan &z_t, const geometry_msgs::Pose &pose);
    double normalDistribution(double x, double mean);
    double numericalIntegration(double mean, double z_max_range, int num_steps);
    void configCallback(sensor_fusion::SensorModelConfig &config, uint32_t level);


    ros::NodeHandle nh;
    Visualizer::Visualizer viz;
    Communication::Subscriber subscriber;
    Communication::Publisher publisher;
    double visualizeRaysPercentage = 5;     // Percentage of rays to visualize

    dynamic_reconfigure::Server<sensor_fusion::SensorModelConfig>* server;
    dynamic_reconfigure::Server<sensor_fusion::SensorModelConfig>::CallbackType f;

    std::vector<Ray> measuredRay;

    // Parameters for the sensor model
    double z_hit;   double sigma_hit;
    double z_short; double lambda_short;
    double z_max;
    double z_rand;
    float z_max_range = 10;

};

#endif // SensorModel_H
