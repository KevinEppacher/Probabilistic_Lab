#!/usr/bin/env python

import rospy
import rospkg
import matplotlib.pyplot as plt
import os
import message_filters
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

class PosePlotter:
    def __init__(self):
        rospy.init_node('pose_plotter', anonymous=True)

        # Subscriber for the combined plot
        self.pose_cov_sub = message_filters.Subscriber('/poseWithCovariance', PoseWithCovarianceStamped)
        self.link_states_sub = message_filters.Subscriber('/gazebo/link_states', LinkStates)
        self.pose_error_sub = message_filters.Subscriber('/pose_error', Float64)

        # Synchronize the three topics with allow_headerless option
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.pose_cov_sub, self.link_states_sub, self.pose_error_sub], 
            queue_size=10, slop=0.1, allow_headerless=True
        )
        self.ts.registerCallback(self.sync_callback)

        # Initialize lists to store data for plotting
        self.estimated_x = []
        self.estimated_y = []
        self.ground_truth_x = []
        self.ground_truth_y = []
        self.pose_errors = []

        # Get the package path
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('sensor_fusion')

        # On shutdown, save the plots
        rospy.on_shutdown(self.save_plots)

    def sync_callback(self, pose_cov_msg, link_states_msg, pose_error_msg):
        # Process synchronized messages
        if len(link_states_msg.pose) > 24:
            rospy.loginfo(f"Received ground truth pose: x={link_states_msg.pose[24].position.x}, y={link_states_msg.pose[24].position.y}")
            self.ground_truth_x.append(link_states_msg.pose[24].position.x)
            self.ground_truth_y.append(link_states_msg.pose[24].position.y)

            rospy.loginfo(f"Received estimated pose: x={pose_cov_msg.pose.pose.position.x}, y={pose_cov_msg.pose.pose.position.y}")
            self.estimated_x.append(pose_cov_msg.pose.pose.position.x)
            self.estimated_y.append(pose_cov_msg.pose.pose.position.y)

            rospy.loginfo(f"Received pose error: {pose_error_msg.data}")
            self.pose_errors.append(pose_error_msg.data)
        else:
            rospy.logwarn("LinkStates does not contain pose[24]")

    def save_plots(self):
        # Save the xy plot
        self.save_plot_xy()

        # Save the pose and error plot
        self.save_plot_pose_error()

    def save_plot_xy(self):
        plt.figure(figsize=(12, 12))
        plt.plot(self.ground_truth_x, self.ground_truth_y, label='Ground Truth Pose', linestyle='-', marker='o')
        plt.plot(self.estimated_x, self.estimated_y, label='Estimated Pose', linestyle='--', marker='x')

        plt.xlabel('X position', fontsize=45)
        plt.ylabel('Y position', fontsize=45)
        plt.legend(fontsize=24)
        plt.title('Ground Truth vs Estimated Pose', fontsize=45)
        plt.grid(True)

        # Increase font size for tick labels
        plt.xticks(fontsize=24)
        plt.yticks(fontsize=24)

        # Define the save path
        save_path = os.path.join(self.package_path, 'measurements')
        if not os.path.exists(save_path):
            os.makedirs(save_path)

        plt.savefig(os.path.join(save_path, 'pose_plotter_xy.png'))
        plt.close()
        rospy.loginfo(f"Plot saved to {os.path.join(save_path, 'pose_plotter_xy.png')}")

    def save_plot_pose_error(self):
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(24, 12))

        # Set larger font sizes
        plt.rcParams.update({'font.size': 45})

        # Plot x positions and error on left side
        ax1.plot(self.ground_truth_x, label='Ground Truth Pose in X', color='red')
        ax1.plot(self.estimated_x, label='MCL Pose in X', color='green')
        ax1.plot(range(len(self.pose_errors)), self.pose_errors, label='Pose Error', color='blue')
        ax1.set_xlabel('Time [s]', fontsize=45)
        ax1.set_ylabel('Pose & Error [m]', fontsize=45)
        ax1.legend(fontsize=24)
        ax1.grid(True)

        # Increase font size for tick labels
        ax1.tick_params(axis='both', which='major', labelsize=40)

        # Plot y positions and error on right side
        ax2.plot(self.ground_truth_y, label='Ground Truth Pose in Y', color='orange')
        ax2.plot(self.estimated_y, label='MCL Pose in Y', color='purple')
        ax2.plot(range(len(self.pose_errors)), self.pose_errors, label='Pose Error', color='blue')
        ax2.set_xlabel('Time [s]', fontsize=45)
        ax2.set_ylabel('Pose & Error [m]', fontsize=45)
        ax2.legend(fontsize=24)
        ax2.grid(True)

        # Increase font size for tick labels
        ax2.tick_params(axis='both', which='major', labelsize=40)

        plt.suptitle('MCL Accuracy Plot', fontsize=48)

        # Define the save path
        save_path = os.path.join(self.package_path, 'measurements')
        if not os.path.exists(save_path):
            os.makedirs(save_path)

        plt.savefig(os.path.join(save_path, 'pose_plotter.png'))
        plt.close()
        rospy.loginfo(f"Plot saved to {os.path.join(save_path, 'pose_plotter.png')}")

if __name__ == '__main__':
    try:
        plotter = PosePlotter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
