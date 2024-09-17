#!/usr/bin/env python

import rospy
from moveit_msgs.msg import DisplayTrajectory
import roboticstoolbox as rtb
import numpy as np

robot = rtb.models.DH.UR5()
class PlannedPathSubscriber:
    def __init__(self):
        self.joint_angles_matrix = []
        # Subscribe to the planned trajectory topic
        rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, self.trajectory_callback)

    def trajectory_callback(self, msg):
        for trajectory in msg.trajectory:
            for joint_trajectory_point in trajectory.joint_trajectory.points:
                joint_positions = joint_trajectory_point.positions[:-1]  # Ignore the last column
                self.joint_angles_matrix.append(joint_positions)

    def forward_kinematics(self):
        print("Forward Kinematics:")
        for joint_positions in self.joint_angles_matrix:
            q = np.array(joint_positions)
            T = robot.fkine(q)
            print("End-effector pose (XYZRPY):", T.t)
            

if __name__ == "__main__":
    rospy.init_node('planned_path_subscriber')
    planned_path_subscriber = PlannedPathSubscriber()

    rospy.spin()

    # After rospy.spin() exits (e.g., on node shutdown), perform forward kinematics
    planned_path_subscriber.forward_kinematics()
