#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class TurtlebotApprox:
    def __init__(self):
        rospy.init_node('turtlebot_approx', anonymous=True)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

    def scan_callback(self, scan_msg):
        # Get the range value at the front (0 degrees) of the robot
        front_distance = scan_msg.ranges[len(scan_msg.ranges)//2]

        # Desired distance to the object (adjust as needed)
        target_distance = 0.30

        # Proportional control to adjust linear velocity based on the distance
        linear_velocity = 0.2 * (front_distance - target_distance)

        # Set a threshold to prevent the robot from getting too close
        if front_distance < target_distance:
            linear_velocity = 0.0

        # Create Twist message and publish
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity
        self.velocity_pub.publish(cmd_vel)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        turtlebot_approx = TurtlebotApprox()
        turtlebot_approx.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down turtlebot_approx node.")
#test