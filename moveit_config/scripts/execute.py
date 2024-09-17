#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import random
import numpy as np

def my_publisher():
    rospy.init_node('execute')
    control_publisher = rospy.Publisher('/ur5_arm_controller/command', JointTrajectory, queue_size=10)
    
    while not rospy.is_shutdown():
        msg = JointTrajectory()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = ''
        msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint','wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        point = JointTrajectoryPoint()
        # j1 = random.random()
        # j2 = random.random()    
        # j3 = random.random()    
        # j4 = random.random()    
        # j5 = random.random()    
        # j6 = random.random()        

        # point.positions = [-2,-2,-2, 2, 2, 2]
        # point.velocities = []
        # point.accelerations = []
        # point.effort = []

        # Read from qhistory.csv and execute the trajectory
        qexecute = np.loadtxt('/home/shobot/sim_ws/src/moveit_config/scripts/joint_angles_24-6-2024.csv', delimiter=',')
        nr, nc = qexecute.shape
        rospy.loginfo("Number of waypoints: %d", nc)

    
        for i in range(nc):
            # Create a new point for each joint configuration
            point = JointTrajectoryPoint()
            point.positions = qexecute[:, i]
            point.velocities = [2,2,2,2,2,2]  # No velocities provided
            point.accelerations = []  # No accelerations provided
            point.time_from_start = rospy.Duration(1.0 * (i + 1))  # Set time_from_start for each point
            msg.points = [point]  # Set the trajectory message to have only the current point

            # Publish the trajectory message
            control_publisher.publish(msg)
            rospy.loginfo("Published waypoint %d", i+1)

            # Wait for the duration of the current trajectory point
            rospy.sleep(1.0)


    
if __name__ == '__main__':
   my_publisher()
