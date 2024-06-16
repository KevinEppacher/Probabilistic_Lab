#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

//Include Libraries
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <random>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>


//Include Custom Classes
#include "Motion_Model.h"
#include "Communication.h"
#include "Visualizer.h"
#include "Functions.h"
#include "Sensor_Model.h"

//Include Structs
#include "../structs/States.h"
#include "../structs/Particle.h"
#include "../structs/Ray.h"


class ParticleFilter 
{
public:
    explicit ParticleFilter(ros::NodeHandle& nodehandler, int quantityParticles = 100);  // Default value for particles
    ~ParticleFilter();

    std::vector<Particle> initializeParticles(const nav_msgs::OccupancyGrid &map);

    std::vector<Particle> estimatePoseWithMCL(const std::vector<Particle>& particles,
                                        const geometry_msgs::Twist& motionCommand,
                                        const sensor_msgs::LaserScan& sensorMeasurement, 
                                        const nav_msgs::OccupancyGrid& map);

    void getNodehanlder(ros::NodeHandle& nodehandler);


private:
    geometry_msgs::PoseArray convertParticlesToPoseArray(const std::vector<Particle> &particles);
    bool isPoseInFreeCell(const geometry_msgs::Pose &pose, const nav_msgs::OccupancyGrid &map);
    std::vector<std::pair<float, float>> findFreeCells(const nav_msgs::OccupancyGrid &map);
    std::vector<Particle> resampleParticles(std::vector<Particle>& particles);

    Particle particle;
    int quantityParticles;
    ros::NodeHandle nh;
    Communication::Publisher publisher;
    Communication::Subscriber subscriber;
    Visualizer::Visualizer visualizer;
    MotionModel motionModel;
    SensorModel sensorModel;
    nav_msgs::OccupancyGrid map;

    float randomOrientation(std::mt19937 &gen);

};

#endif // PARTICLE_FILTER_H
