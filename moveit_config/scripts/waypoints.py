import rospy
from moveit_msgs.msg import DisplayTrajectory

# Callback function to process received messages
def trajectory_callback(msg):
    # Extract waypoints from the executed trajectory
    waypoints = []
    for trajectory in msg.trajectory:
        for point in trajectory.joint_trajectory.points:
            # Extract (x, y, z) coordinates from the trajectory points
            waypoint = point.positions  # Assuming the waypoint is represented by joint positions
            waypoints.append(waypoint)

    # Print the extracted waypoints
    print("Extracted waypoints:", waypoints)

# Initialize ROS node
rospy.init_node('waypoints')

# Subscribe to the executed trajectory topic
rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, trajectory_callback)

# Spin ROS
rospy.spin()
