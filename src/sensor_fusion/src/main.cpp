// main.cpp
#include "ros/ros.h"
#include "Localization.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle nh;

    Localization localization;

    ros::Rate loop_rate(10);  // Set the frequency to 10 Hz

    while (ros::ok()) {
        ros::spinOnce();  // Handle ROS callbacks

        

        loop_rate.sleep();  // Warte, um die Loop-Rate einzuhalten
    }

    return 0;
}
