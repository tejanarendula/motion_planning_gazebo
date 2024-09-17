#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
import numpy as np
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import PointStamped as TFPointStamped


class ObjectDetector:
    def __init__(self):
        rospy.init_node('coordinates')

        # Initialize ROS publishers and subscribers
        self.image_sub = rospy.Subscriber('/camera1/rgb/image_raw', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/camera1/rgb/camera_info', CameraInfo, self.camera_info_callback)
        self.object_pub = rospy.Publisher('/object_coordinates', PointStamped, queue_size=10)

        # Initialize OpenCV
        self.bridge = CvBridge()

        # Initialize TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # Initialize camera intrinsic parameters
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

    def camera_info_callback(self, msg):
        # Extract camera intrinsic parameters
        self.fx = msg.K[0]  # Focal length along the X-axis
        self.fy = msg.K[4]  # Focal length along the Y-axis
        self.cx = msg.K[2]  # Principal point X-coordinate
        self.cy = msg.K[5]  # Principal point Y-coordinate


    def image_callback(self, msg):
        # Check image encoding
        # if msg.encoding != "bgr8":
        #     rospy.logerr("Unsupported image format: {}".format(msg.encoding))
        #     return
        
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Error converting image: {}".format(e))
            return

        # Convert BGR to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define range of target color in HSV
        lower_color = np.array([0, 100, 100])
        upper_color = np.array([10, 255, 255])

        # Threshold the HSV image to get only target colors
        mask = cv2.inRange(hsv_image, lower_color, upper_color)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw contours on the original image
        cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 2)
        
        # Process contours
        for contour in contours:
            # Calculate centroid of contour
            M = cv2.moments(contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Convert centroid from pixel coordinates to 3D pose in camera frame
                depth = self.get_depth(cx, cy)

                if depth is not None:
                    # Print object centroid in camera frame in pixels
                    rospy.loginfo("Object centroid in camera frame: ({}, {}, {})".format(cx, cy, depth))
                    
                    # Convert pixel coordinates to camera frame coordinates
                    x_camera = (cx - self.cx) * depth / self.fx
                    y_camera = (cy - self.cy) * depth / self.fy

                    # Print object centroid in camera frame in meters
                    rospy.loginfo("Object centroid in camera frame (m): ({}, {}, {})".format(x_camera, y_camera, depth))
                    
                    camera_point = PointStamped()
                    camera_point.header.stamp = rospy.Time.now()
                    camera_point.header.frame_id = msg.header.frame_id
                    camera_point.point = Point(x_camera, y_camera, depth)

                    # Convert camera point to world frame
                    world_point = self.transform_point(camera_point, "world")

                    # Publish object coordinates in world frame
                    self.object_pub.publish(world_point)
                    rospy.loginfo("Object centroid in world frame: ({}, {}, {})".format(
                        world_point.point.x, world_point.point.y, world_point.point.z))
                break  # Process only the first contour
        
        # Display the image with contours
        cv2.imshow("Image with Contours", cv_image)
        cv2.waitKey(1)

    def get_depth(self, x, y):
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            rospy.logerr("Camera intrinsic parameters not yet received.")
            return None

        try:
            # Subscribe to depth image
            depth_image = rospy.wait_for_message("/camera1/depth/image_raw", Image, timeout=5.0)

            # Convert ROS depth image to numpy array
            depth_image_data = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")

            # Get depth value at the given pixel coordinates
            depth = depth_image_data[y, x]

            return depth
        except Exception as e:
            rospy.logerr("Error fetching depth data: {}".format(e))
            return None
        
    def transform_point(self, point, target_frame):
        try:
            # Transform the point to the target frame
            tf_point = self.tf_buffer.transform(point, target_frame)
            return tf_point
        except Exception as e:
            rospy.logerr("Error transforming point: {}".format(e))
            return None

    

if __name__ == '__main__':
    try:
        detector = ObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
