#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID
import actionlib_msgs.msg

class ExplorationLogicNode:
    def __init__(self):
        rospy.init_node('exploration_logic_node')

        # Initialize parameters
        self.stuck_threshold = rospy.Duration(5)  # Stuck threshold in seconds
        self.reverse_duration = rospy.Duration(2)  # Reverse duration in seconds

        # Initialize variables
        self.last_odometry_time = rospy.Time.now()

        # Setup subscribers
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)

        # Setup publisher for canceling move_base goals
        self.move_base_cancel_pub = rospy.Publisher('/move_base/cancel', actionlib_msgs.msg.GoalID, queue_size=10)

        # Setup publisher for sending Twist commands to move_base
        self.move_base_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Run the node
        rospy.spin()

    def laser_callback(self, scan):
        # Check for conditions to handle wall
        # Adjust these conditions based on your LDS1 Turtlebot3 laser scan values
        if min(scan.ranges) < 0.5:
            rospy.loginfo("Wall detected. Handling wall condition.")
            self.handle_wall_condition()

    def odometry_callback(self, odom):
        current_time = rospy.Time.now()
        # Check if the robot is stuck
        if current_time - self.last_odometry_time > self.stuck_threshold:
            rospy.loginfo("Robot is stuck. Reversing and changing direction.")
            self.reverse()
            self.spin_90_degrees()
            # Update the last odometry time
            self.last_odometry_time = current_time

    def reverse(self):
        # Send Twist command to reverse the robot
        reverse_cmd = Twist()
        reverse_cmd.linear.x = -0.1  # Adjust the linear velocity as needed
        self.move_base_pub.publish(reverse_cmd)
        rospy.sleep(self.reverse_duration)
        # Stop the robot after reversing
        self.move_base_pub.publish(Twist())

    def spin_90_degrees(self):
        # Send Twist command to spin the robot 90 degrees
        spin_cmd = Twist()
        spin_cmd.angular.z = 0.5  # Adjust the angular velocity as needed
        duration = rospy.Duration(1.8)  # Adjust the duration for approximately 90 degrees rotation
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < duration:
            self.move_base_pub.publish(spin_cmd)
            rospy.sleep(0.1)
        # Stop the robot after spinning
        self.move_base_pub.publish(Twist())

    def handle_wall_condition(self):
        # Cancel move_base goals
        cancel_msg = GoalID()
        self.move_base_cancel_pub.publish(cancel_msg)
        rospy.sleep(1)  # Ensure that move_base has canceled the goal

        # Handle the wall condition (e.g., turn back 40 centimeters, spin 90 degrees)
        self.reverse()
        self.spin_90_degrees()

if __name__ == '__main__':
    exploration_logic_node = ExplorationLogicNode()

