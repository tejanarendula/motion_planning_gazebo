#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from tf2_geometry_msgs import do_transform_pose

def read_joint_angles_from_file(filename):
    """
    Read joint angles from a text file.

    Args:
        filename (str): The path to the text file containing joint angles.

    Returns:
        list: A list of joint angle lists, where each inner list represents a set of joint angles.
    """
    joint_angles_list = []
    with open(filename, 'r') as file:
        for line in file:
            # Split the line by commas and convert each element to float (excluding the last one)
            joint_angles = [float(angle) for angle in line.strip().split(',')[:-1]]
            joint_angles_list.append(joint_angles)
    return joint_angles_list


if __name__ == '__main__':
    rospy.init_node('coordinate_transformer', anonymous=True)

    # Initialize MoveIt
    move_group = MoveGroupCommander("ur5_arm")

    # Create a transform listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Get the target frame from the user
    target_frame = input("Enter the target frame: ")

    # Read joint angles from file
    filename = input("Enter the path to the file containing joint angles: ")
    joint_angles_list = read_joint_angles_from_file(filename)

    for joint_angles in joint_angles_list:
        print("Joint angles:", joint_angles)
        # Set joint angles
        move_group.set_joint_value_target(joint_angles)
        
        move_group.go(wait=True)

        # Get current pose
        current_pose = move_group.get_current_pose().pose

        try:
            # Create a PoseStamped message with the current pose
            pose_in = PoseStamped()
            pose_in.header.frame_id = move_group.get_planning_frame()
            pose_in.pose = current_pose

            # Lookup the transform from the current frame to the target frame
            transformStamped = tf_buffer.lookup_transform(target_frame, move_group.get_planning_frame(), rospy.Time(0), rospy.Duration(3.0))

            # Transform the current pose to the target frame
            pose_out = do_transform_pose(pose_in, transformStamped)

            # Print the transformed pose
            rospy.loginfo("Transformed pose in %s frame:", target_frame)
            rospy.loginfo("Position: %f, %f, %f", pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z)
            rospy.loginfo("Orientation: %f, %f, %f, %f", pose_out.pose.orientation.w, pose_out.pose.orientation.x, pose_out.pose.orientation.y, pose_out.pose.orientation.z)

        except tf2_ros.TransformException as ex:
            rospy.logwarn(str(ex))

    rospy.loginfo("Coordinate transformer shut down")
