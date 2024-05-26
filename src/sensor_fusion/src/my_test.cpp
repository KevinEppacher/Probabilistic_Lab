#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_array_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("pose_array", 10);

    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = "map"; // der relevante Frame, typischerweise "map" oder "odom"

    // Beispiel: Erzeugen von 5 unterschiedlichen Poses
    for (int i = 0; i < 5; i++)
    {
        geometry_msgs::Pose pose;
        pose.position.x = 1.0 + 0.5 * i;
        pose.position.y = 1.0 + 0.2 * i;
        pose.position.z = 0.0;                                      // typischerweise 0 in 2D-Anwendungen
        pose.orientation = tf::createQuaternionMsgFromYaw(0.3 * i); // Rotation angepasst

        // Pose zum PoseArray hinzufügen
        pose_array.poses.push_back(pose);
    }

    ros::Rate rate(1); // 1 Hz
    while (ros::ok())
    {
        pose_array.header.stamp = ros::Time::now(); // Aktualisiere den Zeitstempel des Headers
        pub.publish(pose_array);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
