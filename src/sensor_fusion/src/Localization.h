#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

struct Particle {
    double x, y, theta;  // Particle state variables
    double weight;       // Weight of this particle
};

struct State {
    double x, y, theta;  // State of the robot
    State(double x = 0.0, double y = 0.0, double theta = 0.0) : x(x), y(y), theta(theta) {}
};

class Localization {
public:
    Localization();
    ~Localization();

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

private:
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
};

class ParticleFilter {
public:
    explicit ParticleFilter(int quantityParticles = 100);  // Default value for particles
    ~ParticleFilter();

    std::vector<Particle> initializeParticles(const State& initState);

private:
    int quantityParticles;
};

#endif // LOCALIZATION_H
