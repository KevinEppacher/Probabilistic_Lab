#include "Localization.h"

Localization::Localization() {
    std::cout << "Lokalisierer wurde erstellt" << std::endl;

    // Initialize the subscriber
    odom_sub_ = nh_.subscribe("odom", 1000, &Localization::odomCallback, this);
}

Localization::~Localization() {
    std::cout << "Lokalisierer wird zerstört" << std::endl;
}

void Localization::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Process the incoming message
    std::cout << "Received odometry data: " << std::endl;
    std::cout << "Position (x): " << msg->pose.pose.position.x << std::endl;
    std::cout << "Position (y): " << msg->pose.pose.position.y << std::endl;
    std::cout << "Orientation (z): " << msg->pose.pose.orientation.z << std::endl;
}
