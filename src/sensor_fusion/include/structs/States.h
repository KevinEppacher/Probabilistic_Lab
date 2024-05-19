#ifndef STATES_H
#define STATES_H

struct State 
{
    double x, y, theta;  // State of the robot
    State(double x = 0.0, double y = 0.0, double theta = 0.0) : x(x), y(y), theta(theta) {}
};


#endif // STATES_H