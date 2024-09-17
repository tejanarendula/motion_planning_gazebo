#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from octomap_msgs.msg import Octomap

def update_planning_scene(octomap_msg):
    scene.clear_octomap()
    scene.add_octomap(octomap_msg)

def octomap_callback(msg):
    update_planning_scene(msg)

def go_to_initial_pose():
    group.set_named_target('start')
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

def plan_and_execute():
    # Set the target pose position
    target_pose.position.x = -0.3
    target_pose.position.y = 1.211
    target_pose.position.z = 1.209
    
    # Get the current orientation from the current pose
    current_pose = group.get_current_pose().pose
    target_pose.orientation = current_pose.orientation
    
    group.set_pose_target(target_pose)
    plan = group.go(wait=True)
    if plan:
        rospy.loginfo("Plan found, executing")
    else:
        rospy.loginfo("No plan found")
    group.stop()
    group.clear_pose_targets()

if __name__ == '__main__':
    try:
        rospy.init_node('ur5arm_planner', anonymous=True)
        
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        
        # Define the planning group for the UR5 arm
        group_name = "ur5_arm"
        group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        rospy.Subscriber('/octomap_binary', Octomap, octomap_callback)

        target_pose = geometry_msgs.msg.Pose()

        rospy.sleep(2)
        
        # Go to the initial predefined pose 'start'
        go_to_initial_pose()

        # Plan and execute the movement to the target pose
        plan_and_execute()
        
        moveit_commander.roscpp_shutdown()

    except rospy.ROSInterruptException:
        pass


