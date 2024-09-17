#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

def joint_states_callback(msg):
    # Access joint state information
    positions = msg.position
    rospy.loginfo("Received Joint Positions: %s", positions)

    # Create JointState message
    joint_state_msg = JointState()
    joint_state_msg.header.stamp = rospy.Time.now()
    joint_state_msg.name = ['Revolute_1', 'Revolute_2', 'Revolute_3', 'Revolute_4', 'elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    joint_state_msg.position = positions

    # Publish JointState message
    joint_state_publisher.publish(joint_state_msg)

if __name__ == '__main__':
    rospy.init_node('joint_states')

    # Use the correct topic for your robot. UR5 typically publishes to '/joint_states'
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)

    # Create publishers
    joint_state_publisher = rospy.Publisher('/joint_states', JointState, queue_size=1)
    rospy.spin()
