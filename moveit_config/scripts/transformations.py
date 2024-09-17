#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_geometry_msgs import do_transform_point

if __name__ == '__main__':
    rospy.init_node('transformations', anonymous=True)

    # Create a transform listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Get the target frame from the user
    target_frame = input("Enter the target frame: ")

    while not rospy.is_shutdown():
        # Get user input for coordinates
        x = float(input("Enter x coordinate in the world frame: "))
        y = float(input("Enter y coordinate in the world frame: "))
        z = float(input("Enter z coordinate in the world frame: "))

        # Create a PointStamped message with the input coordinates
        point_in = PointStamped()
        point_in.header.frame_id = "world"
        point_in.point.x = x
        point_in.point.y = y
        point_in.point.z = z

        try:
            # Lookup the transform from the world frame to the target frame
            transformStamped = tf_buffer.lookup_transform(target_frame, "world", rospy.Time(0), rospy.Duration(3.0))

            # Transform the input point to the target frame
            point_out = do_transform_point(point_in, transformStamped)

            # Print the transformed coordinates
            rospy.loginfo("Transformed coordinates in %s frame:", target_frame)
            rospy.loginfo("x: %f, y: %f, z: %f", point_out.point.x, point_out.point.y, point_out.point.z)

        except tf2_ros.TransformException as ex:
            rospy.logwarn(str(ex))

        # Ask if the user wants to continue
        choice = input("Do you want to continue? (y/n): ")
        if choice.lower() != 'y':
            break

    rospy.loginfo("Coordinate transformer shut down")
