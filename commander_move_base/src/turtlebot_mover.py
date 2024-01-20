#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

class TurtlebotMover:
    def __init__(self):
        rospy.init_node('turtlebot_mover', anonymous=True)
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Wait for the action server to come up
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()

    def move_to_goal(self, goal_pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set the goal position
        goal.target_pose.pose.position = Point(*goal_pose[:3])

        # Convert Euler angles to quaternion and set the goal orientation
        quat = quaternion_from_euler(*goal_pose[3:])
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        # Send the goal and wait for result
        rospy.loginfo(f"Moving to goal: {goal_pose}")
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

        # Check if the robot reached the goal
        if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached!")
            return True
        else:
            rospy.logwarn("Failed to reach the goal.")
            return False

    def run(self):
        # Define three different goal positions (x, y, z, roll, pitch, yaw)
        goals = [
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [2.0, 2.0, 0.0, 0.0, 0.0, 1.57],  # 1.57 radians is approximately 90 degrees
            [0.0, 2.0, 0.0, 0.0, 0.0, 3.14]   # 3.14 radians is approximately 180 degrees
        ]

        for goal in goals:
            self.move_to_goal(goal)

if __name__ == '__main__':
    try:
        mover = TurtlebotMover()
        mover.run()
    except rospy.ROSInterruptException:
        pass
