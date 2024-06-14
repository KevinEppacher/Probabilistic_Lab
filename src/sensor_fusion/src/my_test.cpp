#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_fusion/MotionModelConfig.h>
#include <sensor_fusion/SensorModelConfig.h>

class ClassOne
{
public:
    ClassOne(ros::NodeHandle &nh) : nh_(nh)
    {
        dynamic_reconfigure::Server<sensor_fusion::MotionModelConfig>::CallbackType f;
        f = boost::bind(&ClassOne::reconfigureCallback, this, _1, _2);
        server_.setCallback(f);
    }

    void reconfigureCallback(sensor_fusion::MotionModelConfig &config, uint32_t level)
    {
            alpha1 = config.alpha1;
            alpha2 = config.alpha2;
            alpha3 = config.alpha3;
            alpha3 = config.alpha4;
            alpha5 = config.alpha5;
            alpha6 = config.alpha6;

            ROS_INFO("Reconfigure Request: alpha1=%f, alpha2=%f, alpha3=%f, alpha4=%f, alpha5=%f, alpha6=%f",
                     alpha1, alpha2, alpha3, alpha4, alpha5, alpha6);
    }

private:
    ros::NodeHandle nh_;
    double alpha1, alpha2, alpha3, alpha4, alpha5, alpha6;

    dynamic_reconfigure::Server<sensor_fusion::MotionModelConfig> server_{nh_};
};

class ClassTwo
{
public:
    ClassTwo(ros::NodeHandle &nh) : nh_(nh)
    {
        dynamic_reconfigure::Server<sensor_fusion::SensorModelConfig>::CallbackType f;
        f = boost::bind(&ClassTwo::reconfigureCallback, this, _1, _2);
        server_.setCallback(f);
    }

    void reconfigureCallback(sensor_fusion::SensorModelConfig &config, uint32_t level)
    {
        // Handle reconfiguration for ClassTwo
    }

private:
    ros::NodeHandle nh_;
    dynamic_reconfigure::Server<sensor_fusion::SensorModelConfig> server_{nh_};
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh;

    ros::NodeHandle nh1("~class_one");
    ros::NodeHandle nh2("~class_two");

    ClassOne class_one(nh1);
    ClassTwo class_two(nh2);

    ros::spin();
    return 0;
}
