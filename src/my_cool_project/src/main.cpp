#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <my_package/ClassOneConfig.h>
#include <my_package/ClassTwoConfig.h>

class ClassOne {
public:
  ClassOne(ros::NodeHandle &nh) {
    server_.setCallback(boost::bind(&ClassOne::reconfigureCallback, this, _1, _2));
  }

  void reconfigureCallback(my_package::ClassOneConfig &config, uint32_t level) {
    // Handle reconfiguration for ClassOne
  }

private:
  dynamic_reconfigure::Server<my_package::ClassOneConfig> server_;
};

class ClassTwo {
public:
  ClassTwo(ros::NodeHandle &nh) {
    server_.setCallback(boost::bind(&ClassTwo::reconfigureCallback, this, _1, _2));
  }

  void reconfigureCallback(my_package::ClassTwoConfig &config, uint32_t level) {
    // Handle reconfiguration for ClassTwo
  }

private:
  dynamic_reconfigure::Server<my_package::ClassTwoConfig> server_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "my_node");
  ros::NodeHandle nh;

  ClassOne class_one(nh);
  ClassTwo class_two(nh);

  ros::spin();
  return 0;
}
