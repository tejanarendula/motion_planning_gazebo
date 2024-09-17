#!/usr/bin/env python

import rospy
import moveit_commander
from geometry_msgs.msg import Pose

def main():
    rospy.init_node("moveit_smooth_path")
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("ur5_arm")  # Replace "manipulator" with your group name

    # Define the file path
    file_path = '/home/shobot/sim_ws/src/moveit_config/scripts/end_effector_coordinates.txt'

    # Initialize an empty list to store the coordinates
    coordinates_list = []

    # Read the coordinates from the file
    with open(file_path, 'r') as file:
        # Iterate over each line in the file
        for line in file:
            # Split the line into individual coordinates
            coordinates = line.strip().split(',')
            # Convert the coordinates to floats and add them to the list
            try:
                coordinates_list.append([float(coord) for coord in coordinates if coord.strip()])
            except ValueError as e:
                print("Error converting coordinates:", e)

    # Print the list of coordinates
    #print(coordinates_list)
    #waypoints = [(0, 0, 0), (1, 1, 1), (2, 0, 3), (3, 2, 1)]  # Sample waypoints


    scale = 1.0  # Velocity scaling factor

    # Set the maximum velocity scaling factor for the entire trajectory
    group.set_max_velocity_scaling_factor(scale)

    # Set the maximum acceleration scaling factor for the entire trajectory
    group.set_max_acceleration_scaling_factor(scale)

    # Set the reference frame for the waypoints
    group.set_pose_reference_frame("world")  # Assuming "base_link" is your reference frame
    

    waypoints_with_orientation = []
    for waypoint in coordinates_list:
        pose = Pose()
        pose.position.x = waypoint[0]
        pose.position.y = waypoint[1]
        pose.position.z = waypoint[2]
        pose.orientation = group.get_current_pose().pose.orientation # Default orientation

        waypoints_with_orientation.append(pose)

    # Set the start state to the current state
    group.set_start_state_to_current_state()

    # Plan the trajectory
    (plan, fraction) = group.compute_cartesian_path(
        waypoints_with_orientation,  # Waypoints to follow
        0.01,  # Eef_step
        0.0,   # Jump_threshold
        True   # Avoid_collisions
    )

    # Execute the trajectory
    group.execute(plan)
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass