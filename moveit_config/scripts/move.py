#!/usr/bin/env python

import rospy
import sys
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import Pose

def move_arm():
    rospy.init_node('move')

    # Initialize MoveItCommander
    moveit_commander.roscpp_initialize(sys.argv)

    # Set up the MoveGroupInterface for the arm and gripper
    arm_group = moveit_commander.MoveGroupCommander("ur5_arm")
    gripper_group = moveit_commander.MoveGroupCommander("grippers")

    # Set the target pose for the arm
    arm_group.set_named_target("grasp")

    # Plan the trajectory
    success, arm_plan, *_ = arm_group.plan()

    # Execute the trajectory
    if success:
        arm_group.execute(arm_plan, wait=True)
    else:
        rospy.logerr("Planning failed")

    rospy.sleep(3.0)

    pose1 = arm_group.get_current_pose().pose
    
    # Set the target pose for the arm
    target_pose1 = Pose()
    target_pose1.orientation = pose1.orientation
    target_pose1.position.x = -0.3
    target_pose1.position.y = 1.21
    target_pose1.position.z = 1.20
    arm_group.set_pose_target(target_pose1)

    # Plan the trajectory
    success, arm_plan, *_ = arm_group.plan()

    # Execute the trajectory
    if success:
        arm_group.execute(arm_plan, wait=True)
    else:
        rospy.logerr("Planning failed")

    rospy.loginfo("Arm movement complete.")

    # Clean up
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        move_arm()
    except rospy.ROSInterruptException:
        rospy.logerr("Interrupted")



