#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class InterruptNode:
    def __init__(self):
        rospy.init_node('interrupt_node')
        self.stuck_threshold = 3.0  # Stuck threshold in seconds
        self.last_pose_time = rospy.Time.now()

        # Subscribe to TurtleBot3's pose topic
        rospy.Subscriber('/odom_pose', PoseStamped, self.pose_callback)

        # Create a publisher to trigger the interrupt
        self.interrupt_pub = rospy.Publisher('/interrupt', Bool, queue_size=1)

    def pose_callback(self, pose_msg):
        current_time = rospy.Time.now()
        if (current_time - self.last_pose_time).to_sec() > self.stuck_threshold:
            # TurtleBot3 is stuck, trigger interrupt
            self.interrupt_pub.publish(Bool(True))

        self.last_pose_time = current_time

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    interrupt_node = InterruptNode()
    interrupt_node.run()

