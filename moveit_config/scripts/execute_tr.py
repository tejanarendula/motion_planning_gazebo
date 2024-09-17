#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

def my_publisher():
    rospy.init_node('execute')
    control_publisher = rospy.Publisher('/ur5_arm_controller/command', JointTrajectory, queue_size=10)
    
    # Load the joint angles from the CSV file
    qexecute = np.loadtxt('/home/shobot/sim_ws/src/moveit_config/scripts/joint_angles_apf_prm.csv', delimiter=',')
    
    # Get every alternate row starting from the first row
    qexecute = qexecute[::2]
    
    # Get the number of waypoints
    nr, nc = qexecute.shape
    rospy.loginfo("Number of waypoints: %d", nr)
    
    msg = JointTrajectory()

    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = ''
    msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    for i in range(nr):
        # Create a new point for each joint configuration
        point = JointTrajectoryPoint()
        point.positions = qexecute[i, :]
        point.velocities = []  # Set velocities
        point.accelerations = []  # No accelerations provided
        point.time_from_start = rospy.Duration(1.0 * (i + 1))  # Set time_from_start for each point
        msg.points = [point]  # Set the trajectory message to have only the current point

        # Publish the trajectory message
        control_publisher.publish(msg)
        rospy.loginfo("Published waypoint %d", i + 1)

        # Wait for the duration of the current trajectory point
        rospy.sleep(1.0)
    
    # Exit the loop after publishing all waypoints
    rospy.loginfo("Finished publishing all waypoints.")

if __name__ == '__main__':
    my_publisher()
