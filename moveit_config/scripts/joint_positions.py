#!/usr/bin/env python3

import rospy
from moveit_msgs.msg import DisplayTrajectory

class PlannedPathSubscriber:
    def __init__(self):
        self.joint_angles_matrix = []

        # Subscribe to the planned trajectory topic
        rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, self.trajectory_callback)

    def trajectory_callback(self, msg):
        for trajectory in msg.trajectory:
            for joint_trajectory_point in trajectory.joint_trajectory.points:
                joint_positions = joint_trajectory_point.positions
                self.joint_angles_matrix.append(joint_positions)

        # Print or store joint angles after receiving all trajectory data
        self.print_joint_angles()
        self.store_joint_angles("joint_angles_matrix.txt")

    def print_joint_angles(self):
        print("Joint Angles Matrix:")
        for row in self.joint_angles_matrix:
            print(row)

    def store_joint_angles(self, filename):
        with open(filename, 'w') as f:
            for row in self.joint_angles_matrix:
                f.write(','.join(str(angle) for angle in row) + '\n')

if __name__ == "__main__":
    rospy.init_node('planned_path_subscriber')
    planned_path_subscriber = PlannedPathSubscriber()

    rospy.spin()
