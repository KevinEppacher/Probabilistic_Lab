#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Float64
import math

class PoseErrorCalculator:
    def __init__(self):
        rospy.init_node('pose_error_calculator', anonymous=True)

        # Subscriber
        self.pose_cov_sub = rospy.Subscriber('/poseWithCovariance', PoseWithCovarianceStamped, self.pose_cov_callback)
        self.link_states_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_states_callback)

        # Publisher
        self.error_pub = rospy.Publisher('/pose_error', Float64, queue_size=10)

        # Initialize poses
        self.pose_cov = None
        self.pose_link = None

    def pose_cov_callback(self, msg):
        self.pose_cov = msg.pose.pose
        self.calculate_and_publish_error()

    def link_states_callback(self, msg):
        if len(msg.pose) > 24:
            self.pose_link = msg.pose[24]
            self.calculate_and_publish_error()
        else:
            rospy.logwarn("LinkStates does not contain pose[24]")

    def calculate_and_publish_error(self):
        if self.pose_cov is None or self.pose_link is None:
            return
        
        # Calculate the Euclidean distance
        dx = self.pose_cov.position.x - self.pose_link.position.x
        dy = self.pose_cov.position.y - self.pose_link.position.y
        dz = self.pose_cov.position.z - self.pose_link.position.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        # Publish the error
        self.error_pub.publish(distance)
        rospy.loginfo("Pose error: {}".format(distance))

if __name__ == '__main__':
    try:
        calculator = PoseErrorCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
