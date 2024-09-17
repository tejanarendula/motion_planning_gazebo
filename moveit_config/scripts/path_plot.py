#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def read_coordinates_from_file(filename):
    """
    Read coordinates from a text file.

    Args:
        filename (str): The path to the text file containing coordinates.

    Returns:
        list: A list of tuples, where each tuple represents a set of coordinates (x, y, z).
    """
    coordinates_list = []
    with open(filename, 'r') as file:
        for line in file:
            # Split the line by tabs, remove leading/trailing whitespace
            parts = line.strip().split('\t')
            # Check if there are valid numerical values
            if len(parts) == 3:
                try:
                    # Convert the coordinates to float and append to the list
                    coordinates = [float(coord) for coord in parts]
                    coordinates_list.append(tuple(coordinates))
                except ValueError:
                    # Skip lines with invalid numerical values
                    pass
    return coordinates_list

if __name__ == '__main__':
    rospy.init_node('transformations', anonymous=True)

    # Create a transform listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Get the target frame from the user
    target_frame = input("Enter the target frame: ")

    # Read coordinates from file
    filename = input("Enter the path to the file containing coordinates: ")
    coordinates_list = read_coordinates_from_file(filename)

    # Perform transformations and store transformed coordinates
    transformed_coordinates = []
    for coordinates in coordinates_list:
        # Create a PointStamped message with the input coordinates
        point_in = PointStamped()
        point_in.header.frame_id = "world"
        point_in.point.x, point_in.point.y, point_in.point.z = coordinates

        try:
            # Lookup the transform from the world frame to the target frame
            transformStamped = tf_buffer.lookup_transform(target_frame, "world", rospy.Time(0), rospy.Duration(3.0))

            # Transform the input point to the target frame
            point_out = do_transform_point(point_in, transformStamped)

            # Store the transformed coordinates
            transformed_coordinates.append((point_out.point.x, point_out.point.y, point_out.point.z))

        except tf2_ros.TransformException as ex:
            rospy.logwarn(str(ex))

    # Print the list of transformed coordinates
    print("Transformed Coordinates:")
    for idx, coord in enumerate(transformed_coordinates, start=1):
        print(f"Point {idx}: {coord}")

    # Plot the transformed coordinates in a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Extract x, y, z coordinates from the transformed coordinates
    x_coordinates, y_coordinates, z_coordinates = zip(*transformed_coordinates)

    # Plot the coordinates and highlight them with markers
    ax.plot(x_coordinates, y_coordinates, z_coordinates, color='b', marker='o', markersize=5)

    # Set labels and title
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.set_title('Path from Start to End Coordinates')

    # Show the plot
    plt.show()

    rospy.loginfo("Transformation and plotting completed.")
