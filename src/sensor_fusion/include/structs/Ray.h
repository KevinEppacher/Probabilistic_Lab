#ifndef RAY_H
#define RAY_H

#include <geometry_msgs/Pose.h>

struct Ray
{
    geometry_msgs::Point origin, end;
    double angle;
    double length;
};

#endif // RAY_H