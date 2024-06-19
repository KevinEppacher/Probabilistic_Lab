#include <ros/ros.h>
#include <random>
#include <vector>

// Struktur für Partikel
struct Particle
{
    double x;      // x-Position
    double y;      // y-Position
    double theta;  // Orientierung
    double weight; // Gewicht
};

// Funktion zum Resampling
std::vector<Particle> resampleParticles(const std::vector<Particle> &particles)
{
    std::vector<Particle> resampledParticles;
    int numParticles = particles.size();
    std::vector<double> weights;

    for (const auto &particle : particles)
    {
        weights.push_back(particle.weight);
    }

    // Normalisiere Gewichte
    double sumWeights = std::accumulate(weights.begin(), weights.end(), 0.0);

    for (auto &weight : weights)
    {
        weight /= sumWeights;
    }

    // Erstelle einen Diskreten Verteilungs-Generator basierend auf den Gewichten
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> d(weights.begin(), weights.end());

    for (int i = 0; i < numParticles; ++i)
    {
        int index = d(gen);
        resampledParticles.push_back(particles[index]);
    }

    return resampledParticles;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "particle_filter_resampling");
    ros::NodeHandle nh;

    // Beispiel-Partikel
    std::vector<Particle> particles = {
        {1.0, 2.0, 0.5, 0.1},
        {2.0, 3.0, 1.0, 0.2},
        {3.0, 1.0, 1.5, 0.3},
        {4.0, 2.0, 0.2, 0.4}};

    std::vector<Particle> resampledParticles = resampleParticles(particles);

    while (ros::ok())
    {
        // Ausgabe der resampelten Partikel
        for (const auto &particle : resampledParticles)
        {
            ROS_INFO("x: %f, y: %f, theta: %f, weight: %f", particle.x, particle.y, particle.theta, particle.weight);
        }

        ROS_INFO("\n");
    }

    return 0;
}

// #include <ros/ros.h>
// #include <dynamic_reconfigure/server.h>
// #include <sensor_fusion/MotionModelConfig.h>
// #include <sensor_fusion/SensorModelConfig.h>

// class ClassOne
// {
// public:
//     ClassOne(ros::NodeHandle &nh) : nh_(nh)
//     {
//         dynamic_reconfigure::Server<sensor_fusion::MotionModelConfig>::CallbackType f;
//         f = boost::bind(&ClassOne::reconfigureCallback, this, _1, _2);
//         server_.setCallback(f);
//     }

//     void reconfigureCallback(sensor_fusion::MotionModelConfig &config, uint32_t level)
//     {
//             alpha1 = config.alpha1;
//             alpha2 = config.alpha2;
//             alpha3 = config.alpha3;
//             alpha3 = config.alpha4;
//             alpha5 = config.alpha5;
//             alpha6 = config.alpha6;

//             ROS_INFO("Reconfigure Request: alpha1=%f, alpha2=%f, alpha3=%f, alpha4=%f, alpha5=%f, alpha6=%f",
//                      alpha1, alpha2, alpha3, alpha4, alpha5, alpha6);
//     }

// private:
//     ros::NodeHandle nh_;
//     double alpha1, alpha2, alpha3, alpha4, alpha5, alpha6;

//     dynamic_reconfigure::Server<sensor_fusion::MotionModelConfig> server_{nh_};
// };

// class ClassTwo
// {
// public:
//     ClassTwo(ros::NodeHandle &nh) : nh_(nh)
//     {
//         dynamic_reconfigure::Server<sensor_fusion::SensorModelConfig>::CallbackType f;
//         f = boost::bind(&ClassTwo::reconfigureCallback, this, _1, _2);
//         server_.setCallback(f);
//     }

//     void reconfigureCallback(sensor_fusion::SensorModelConfig &config, uint32_t level)
//     {
//         // Handle reconfiguration for ClassTwo
//     }

// private:
//     ros::NodeHandle nh_;
//     dynamic_reconfigure::Server<sensor_fusion::SensorModelConfig> server_{nh_};
// };

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "my_node");
//     ros::NodeHandle nh;

//     ros::NodeHandle nh1("~class_one");
//     ros::NodeHandle nh2("~class_two");

//     ClassOne class_one(nh1);
//     ClassTwo class_two(nh2);

//     ros::spin();
//     return 0;
// }
