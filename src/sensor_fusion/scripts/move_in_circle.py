#!/usr/bin/env python

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

def movebase_client():
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()
    
    # Define the radius of the circle
    radius = 1.5  # meters
    # Define the number of waypoints
    num_waypoints = 8
    # Calculate the angular increment
    angle_increment = 2 * math.pi / num_waypoints

    # Define the position offset (center of the circle)
    offset_x = -2.5 # meters
    offset_y = 2.5  # meters

    # Create a list to hold the waypoints
    waypoints = []

    for i in range(num_waypoints):
        # Calculate the x and y position of the waypoint
        theta = i * angle_increment
        x = offset_x + radius * math.cos(theta)
        y = offset_y + radius * math.sin(theta)
        # Create a MoveBaseGoal and set the goal position
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(x, y, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
        waypoints.append(goal)

    while not rospy.is_shutdown():
        for waypoint in waypoints:
            # Sends the goal to the action server.
            client.send_goal(waypoint)
            # Waits for the server to finish performing the action.
            wait = client.wait_for_result()

            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                # Result of executing the action
                rospy.loginfo("Reached waypoint: ({}, {})".format(waypoint.target_pose.pose.position.x, waypoint.target_pose.pose.position.y))

if __name__ == '__main__':
    try:
        rospy.init_node('move_base_circle')
        movebase_client()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
