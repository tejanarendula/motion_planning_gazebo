#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
import geometry_msgs.msg 
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from std_msgs.msg import String
import sys
import moveit_commander
from shape_msgs.msg import SolidPrimitive

# Global variables to store the pose of the object
object_pose = geometry_msgs.msg.PoseStamped()
object_name = ""

# Map to store dimensions for different objects
object_dimensions_map = {
    "box_red": [0.04, 0.12, 0.18],
    "box_green": [0.04, 0.12, 0.12],
    "box_blue": [0.04, 0.2, 0.15],
    "box_red_clone": [0.04, 0.12, 0.18],
    "box_green_clone": [0.04, 0.12, 0.12],
    "box_blue_clone": [0.04, 0.2, 0.1]
}

# Callback function for model states
def model_states_callback(msg):
    global object_pose
    global object_name

    # Find the index of the object in the model states message
    object_index = -1
    for i, name in enumerate(msg.name):
        if name == object_name:
            object_index = i
            break

    # If the object is found, extract its pose
    if object_index != -1:
        object_pose = msg.pose[object_index]
        rospy.loginfo("Object Pose:")
        rospy.loginfo("Position (x, y, z): %f, %f, %f", object_pose.position.x, object_pose.position.y, object_pose.position.z)
        rospy.loginfo("Orientation (x, y, z, w): %f, %f, %f, %f", object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z, object_pose.orientation.w)
        
        # Check if object name exists in the dimensions map
        if object_name in object_dimensions_map:
            # Get dimensions from map
            # Add the missing import for SolidPrimitive
            # initialize robot commander object and planning scene object
            
            robot = moveit_commander.RobotCommander()
            scene = moveit_commander.PlanningSceneInterface()
            
            dimensions = object_dimensions_map[object_name]
            print(dimensions)
            object_pose.header.frame_id = "world"
            name = object_name
            scene.add_box(name, object_pose, dimensions)
            # Add the box as a collision object
            # collision_object = CollisionObject()
            # collision_object.header.frame_id = "world"
            # collision_object.id = object_name

            # primitive = SolidPrimitive()
            # primitive.type = primitive.BOX
            # primitive.dimensions = dimensions

            # collision_object.primitives.append(primitive)
            # collision_object.primitive_poses.append(object_pose)
            # collision_object.operation = collision_object.ADD

            # # Apply the collision object to the planning scene
            # planning_scene_interface.applyCollisionObject(collision_object)
        else:
            rospy.logerr("Dimensions not found for object '%s'", object_name)
    else:
        rospy.logerr("Object '%s' not found in model states!", object_name)

def main():
    global object_name
    global planning_scene_interface

    # Initialize ROS node
    rospy.init_node("object_pose_collision", anonymous=True)

    # Initialize planning scene interface
    planning_scene_interface = PlanningScene()

    # Prompt user to input the name of the object
    object_name = input("Enter the name of the object: ")

    # Subscribe to model states topic
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)

    # Spin and wait for callbacks
    rospy.spin()

if __name__ == "__main__":
    main()
