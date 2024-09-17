#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

if __name__ == '__main__':
    rospy.init_node('minimal_joint_states_publisher')
    joint_state_publisher = rospy.Publisher('/joint_states', JointState, queue_size=1)
    joint_state_msg = JointState()
    joint_state_msg.header.stamp = rospy.Time.now()
    joint_state_msg.name = ['shoulder_pan_joint']
    joint_state_msg.position = [0.0]
    
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
    	joint_state_publisher.publish(joint_state_msg)
    	rate.sleep()
    
