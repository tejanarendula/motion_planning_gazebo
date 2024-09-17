#!/usr/bin/env python3

import subprocess
import time

# Define the path to your ROS setup file
ros_setup_file = "/opt/ros/noetic/setup.bash"  # Update this path according to your ROS distro

# Define the path to your workspace setup file
workspace_setup_file = "/home/shobot/sim_ws/devel/setup.bash"

# Define the path to your .launch file
launch_file_path = "/home/shobot/sim_ws/src/moveit_config/launch/arena_1.launch"

# Define the ROS package and node
ros_package = "moveit_config"
ros_node = "pick_n_place"

# Function to source the ROS and workspace setup files
def source_setup_files():
    try:
        # Construct the command to source both ROS and workspace setup files
        command = f"source {ros_setup_file} && source {workspace_setup_file}"
        subprocess.run(command, shell=True, check=True, executable='/bin/bash')
        print("Successfully sourced ROS and workspace setup files.")
    except subprocess.CalledProcessError as e:
        print(f"An error occurred while sourcing setup files: {e}")

# Function to run the .launch file
def run_ros_launch():
    try:
        # Construct the command to run roslaunch
        command = f"roslaunch {launch_file_path}"
        
        # Use subprocess to run the command in a new shell
        subprocess.Popen(command, shell=True, executable='/bin/bash')
        print(f"Successfully launched {launch_file_path}")
    except subprocess.CalledProcessError as e:
        print(f"An error occurred while launching {launch_file_path}: {e}")

# Function to run the ROS node
def run_ros_node():
    try:
        # Construct the command to run the node
        command = f"rosrun {ros_package} {ros_node}"
        
        # Use subprocess to run the command in a new shell
        subprocess.run(command, shell=True, check=True, executable='/bin/bash')
        print(f"Successfully launched {ros_node} in package {ros_package}")
    except subprocess.CalledProcessError as e:
        print(f"An error occurred while launching {ros_node}: {e}")

# Main function to execute all tasks
if __name__ == "__main__":
    # Source ROS and workspace setup files
    source_setup_files()
    
    # Run the ROS launch file
    run_ros_launch()
    
    # Wait for 15 seconds (if needed)
    time.sleep(15)
    
    # Run the ROS node
    run_ros_node()
