#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "Localization.h"
#include "Particle_Filter.h"

// Forward declaration of State
class State;


class MotionModel 
{
public:
    MotionModel();
    ~MotionModel();

    geometry_msgs::Twist sampleMotionModel(const State& state, const State& control);

private:

};

#endif // MOTION_MODEL_H
