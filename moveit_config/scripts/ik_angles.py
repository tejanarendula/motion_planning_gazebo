#!/usr/bin/env python

import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import PoseStamped

rospy.init_node('moveit_example')

# Create MoveIt! objects
robot_commander = RobotCommander()
move_group = MoveGroupCommander('ur5_arm')

# Set the target pose
target_pose = PoseStamped()
target_pose.header.frame_id = 'world'
target_pose.pose.position.x = 1.0
target_pose.pose.position.y = 0.0
target_pose.pose.position.z = 0.8
# Set the orientation as needed
# target_pose.pose.orientation = ...

# Set the target pose for planning
move_group.set_pose_target(target_pose)

# Plan the trajectory
plan, planning_result = move_group.plan()

# Check if planning was successful
if planning_result and planning_result == move_group.SUCCESS:
    # Fetch the joint values from the plan
    target_joint_angles = plan.joint_trajectory.points[-1].positions
    print("Target Joint Angles:", target_joint_angles)
else:
    print("Planning failed. Planning result:", planning_result)

