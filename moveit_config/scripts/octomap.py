#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

#global variable for the object pose
box_pose = geometry_msgs.msg.PoseStamped()

# Callback function for handling the model states
def model_states_callback(model_states):
    global box_pose
    # Find the index of the object in the model states message
    object_index = -1
    for i, name in enumerate(model_states.name):
        print(enumerate(model_states.name))
        if name == "box_red":  
            object_index = i  
            # If the object is found, extract its pose
            box_pose = model_states.pose[object_index]
            print(box_pose)
        break
        rospy.loginfo("Box Pose: {}".format(box_pose))
        


def add_box():
    global box_pose
    # initialize the moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("octomap", anonymous=True)

    # initialize robot commander object and planning scene object
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "ur5_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    # Wait for the box pose to be populated
    # while box_pose.pose.position.x == 0.0:
    #     rospy.sleep(0.1)

    # Add object to planning scene
    box_pose.header.frame_id = "world"
    # box_pose.pose.orientation.w = 0.5
    # box_pose.pose.orientation.z = 0.5
    # box_pose.pose.position.x = -0.2722
    # box_pose.pose.position.y = 1.2424
    # box_pose.pose.position.z = 1.4727
    box_name = "box_red"
    scene.add_box(box_name, box_pose, size=(0.04, 0.12, 0.18))

if __name__ == '__main__':
    
    # Subscribe to Gazebo model states topic
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    add_box()
    


