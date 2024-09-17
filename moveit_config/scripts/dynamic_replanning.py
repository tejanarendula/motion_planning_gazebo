#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from octomap_msgs.msg import Octomap

# Global flag to indicate environment change
environment_changed = False

def update_planning_scene(octomap_msg):
    global environment_changed
    rospy.loginfo("Updating planning scene with new octomap")
    scene.clear_octomap()
    scene.add_octomap(octomap_msg)
    environment_changed = True

def octomap_callback(msg):
    update_planning_scene(msg)

def go_to_initial_pose():
    rospy.loginfo("Going to initial pose 'start'")
    group.set_named_target('start')
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    if plan:
        rospy.loginfo("Successfully moved to initial pose")
    else:
        rospy.logerr("Failed to move to initial pose")

def plan_and_execute():
    rospy.loginfo("Planning and executing to target pose")
    try:
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
            rospy.loginfo("Plan found and executed successfully")
        else:
            rospy.logerr("No plan found")
  
    except Exception as e:
        rospy.logerr(f"Exception occurred during planning and execution: {e}")
    finally:
        group.stop()
        group.clear_pose_targets()

def replan_and_execute():
    rospy.loginfo("Re-planning and executing to target pose")
    attempts = 5
    for attempt in range(attempts):
        try:
            rospy.loginfo(f"Re-planning attempt {attempt + 1}/{attempts}")

            # Get the current pose of the robot
            current_pose = group.get_current_pose().pose
            rospy.loginfo(f"Current pose: {current_pose}")

            # Set the target pose position
            target_pose.position.x = -0.3
            target_pose.position.y = 1.211
            target_pose.position.z = 1.209

            # Use a default or specified orientation (identity quaternion for simplicity)
            # target_pose.orientation.w = 1.0
            # target_pose.orientation.x = 0.0
            # target_pose.orientation.y = 0.0
            # target_pose.orientation.z = 0.0

            group.set_pose_target(target_pose)
            plan = group.go(wait=True)
            if plan:
                rospy.loginfo("Re-plan found and executed successfully")
                return
            else:
                rospy.logwarn("Re-plan attempt failed")
        except Exception as e:
            rospy.logerr(f"Exception occurred during re-planning and execution: {e}")
    rospy.logerr("Failed to find a valid path after multiple re-planning attempts, returning to initial pose")
    go_to_initial_pose()

def check_environment_and_replan():
    global environment_changed
    if environment_changed:
        rospy.loginfo("Environment change detected, waiting for octomap update...")
        rospy.sleep(2)  # Wait for 2 seconds to ensure octomap is fully updated
        environment_changed = False
        replan_and_execute()

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

        # Monitor environment and replan if necessary
        rate = rospy.Rate(1)  # Check for changes at 1 Hz
        while not rospy.is_shutdown():
            check_environment_and_replan()
            rate.sleep()

        moveit_commander.roscpp_shutdown()

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled exception: {e}")
