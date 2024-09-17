#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point

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

    # Open a file to store transformed coordinates
    output_filename = 'transformed_coordinates.txt'
    output_file = open(output_filename, 'w')

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

            # Write the transformed coordinates to the output file
            output_file.write("Transformed coordinates in %s frame:\n" % target_frame)
            output_file.write("x: %f, y: %f, z: %f\n" % (point_out.point.x, point_out.point.y, point_out.point.z))
            output_file.write("\n")

        except tf2_ros.TransformException as ex:
            rospy.logwarn(str(ex))

    # Close the output file
    output_file.close()

    rospy.loginfo("Transformed coordinates saved to %s", output_filename)
