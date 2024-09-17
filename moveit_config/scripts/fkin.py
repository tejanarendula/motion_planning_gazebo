#!/usr/bin/env python3

import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from geometry_msgs.msg import PoseStamped

def read_joint_angles_from_file(filename):
    """
    Read joint angles from a text file.

    Args:
        filename (str): The path to the text file containing joint angles.

    Returns:
        list: A list of lists, where each inner list represents a set of joint angles.
    """
    joint_angles_list = []
    with open(filename, 'r') as file:
        for line in file:
            # Split the line by commas, convert each element to float, and ignore the last element
            joint_angles = [float(angle) for angle in line.strip().split(',')[:-1]]
            # Append a 0 to represent the last joint angle
            joint_angles.append(0.0)
            joint_angles_list.append(joint_angles)
    return joint_angles_list

if __name__ == '__main__':
    rospy.init_node('forward_kinematics', anonymous=True)

    # Initialize MoveIt
    robot_commander = RobotCommander()
    move_group = MoveGroupCommander("ur5_arm")

    # Get the joint names
    joint_names = move_group.get_active_joints()

    # Get the end effector link
    end_effector_link = move_group.get_end_effector_link()

    # Read joint angles from file
    filename = input("Enter the path to the file containing joint angles: ")
    joint_angles_list = read_joint_angles_from_file(filename)

    # Open a file to store end effector coordinates
    output_filename = 'end_effector_coordinates.txt'
    output_file = open(output_filename, 'w')

    # Calculate forward kinematics for each set of joint angles
    fk_results = []
    for joint_angles in joint_angles_list:
        # Set joint values
        move_group.set_joint_value_target(joint_angles)

        # Calculate forward kinematics
        move_group.go(wait=True)
        fk_result = move_group.get_current_pose(end_effector_link)
        fk_results.append(fk_result)

    # Store end effector coordinates in the file as a matrix
    #output_file.write("End Effector Coordinates (Position):\n")
    #output_file.write("x\t y\t z\n")
    for fk_result in fk_results:
        output_file.write("%f\t %f\t %f\n" % (fk_result.pose.position.x, fk_result.pose.position.y, fk_result.pose.position.z))
    output_file.write("\n")

    # Close the output file
    output_file.close()

    rospy.loginfo("End effector coordinates saved to %s", output_filename)
