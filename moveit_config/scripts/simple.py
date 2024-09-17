#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def simple_trajectory_publisher():
    rospy.init_node('execute')
    control_publisher = rospy.Publisher('/ur5_arm_controller/command', JointTrajectory, queue_size=10)
    
    rospy.sleep(1)  # Give the publisher time to connect
    
    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    point = JointTrajectoryPoint()
    point.positions = [0, -1.57, 1.57, 0, 0, 0]  # Set some specific joint angles
    point.velocities = [0, 0, 0, 0, 0, 0]
    point.time_from_start = rospy.Duration(3.0)
    
    msg.points = [point]

    control_publisher.publish(msg)
    rospy.loginfo("Published simple trajectory command")
    
    rospy.spin()  # Keep the node running

if __name__ == '__main__':
    simple_trajectory_publisher()

