import rospy
import sys
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

def read_coordinates_from_file(filename):
    """
    Read end-effector coordinates from a text file.

    Args:
        filename (str): The path to the text file containing end-effector coordinates.

    Returns:
        list: A list of tuples, where each tuple represents a set of end-effector coordinates (x, y, z).
    """
    coordinates_list = []
    with open(filename, 'r') as file:
        for line in file:
            parts = line.strip().split(',')  # Split by tabs instead of commas
            if len(parts) == 3:
                try:
                    coordinates = tuple(float(coord) for coord in parts)
                    coordinates_list.append(coordinates)
                except ValueError:
                    pass
            else:
                rospy.logwarn("Ignoring line with invalid format: %s", line.strip())
    return coordinates_list

def interpolate_waypoints(waypoints, num_interpolation_points):
    """
    Interpolate additional waypoints between given waypoints.

    Args:
        waypoints (list): List of tuples representing end-effector coordinates.
        num_interpolation_points (int): Number of additional points to interpolate between each pair of waypoints.

    Returns:
        list: Interpolated waypoints including the original ones.
    """
    interpolated_waypoints = []
    for i in range(len(waypoints) - 1):
        for j in range(num_interpolation_points):
            ratio = (j + 1) / (num_interpolation_points + 1)
            interpolated_point = tuple(
                waypoints[i][k] + ratio * (waypoints[i + 1][k] - waypoints[i][k]) for k in range(3)
            )
            interpolated_waypoints.append(interpolated_point)
    return waypoints + interpolated_waypoints

def create_trajectory(arm_group, waypoints):
    """
    Create a trajectory to move the arm through the given waypoints.

    Args:
        arm_group (MoveGroupCommander): The MoveGroupCommander for the arm.
        waypoints (list): List of tuples representing end-effector coordinates.

    Returns:
        RobotTrajectory: The generated trajectory.
    """
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = arm_group.get_active_joints()
    for waypoint in waypoints:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = waypoint
        pose.orientation = arm_group.get_current_pose().pose.orientation
        arm_group.set_pose_target(pose)
        plan = arm_group.plan()
        if plan[0]:
            joint_point = JointTrajectoryPoint()
            joint_point.positions = plan[1].joint_trajectory.points[-1].positions
            trajectory.joint_trajectory.points.append(joint_point)
    return trajectory


def execute_trajectory(arm_group, trajectory):
    """
    Execute the trajectory.

    Args:
        arm_group (MoveGroupCommander): The MoveGroupCommander for the arm.
        trajectory (RobotTrajectory): The trajectory to execute.
    """
    arm_group.execute(trajectory)

def move_arm():
    # Initialize node
    rospy.init_node('move')

    # Initialize MoveItCommander
    moveit_commander.roscpp_initialize(sys.argv)

    # Set up the MoveGroupInterface for the arm
    arm_group = moveit_commander.MoveGroupCommander("ur5_arm")

    # Set the target pose for the arm
    arm_group.set_named_target("start")

    # Plan and execute to start position
    arm_group.go()

    rospy.sleep(3.0)

    # Read end-effector coordinates from file
    filename = input("Enter the path to the file containing end-effector coordinates: ")
    coordinates_list = read_coordinates_from_file(filename)

    # Interpolate waypoints
    interpolated_waypoints = interpolate_waypoints(coordinates_list, num_interpolation_points=5)

    # Create and execute trajectory
    trajectory = create_trajectory(arm_group, interpolated_waypoints)
    execute_trajectory(arm_group, trajectory)

    rospy.loginfo("Arm movement complete.")

    # Clean up
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        move_arm()
    except rospy.ROSInterruptException:
        rospy.logerr("Interrupted")
