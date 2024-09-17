#!/usr/bin/env python
import rospy, sys, numpy as np
import geometry_msgs.msg
import moveit_msgs.msg
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import Empty

def gripper_status(msg):
    if msg.data:
        return True
        # print('gripper status = {}'.format(msg.data))

def gripper_on():
    # Wait till the srv is available
    rospy.wait_for_service('/urdF/vacuum_gripper/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/urdF/vacuum_gripper/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def gripper1_on():
    # Wait till the srv is available
    rospy.wait_for_service('/urdF/vacuum_gripper1/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/urdF/vacuum_gripper1/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def gripper2_on():
    # Wait till the srv is available
    rospy.wait_for_service('/urdF/vacuum_gripper2/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/urdF/vacuum_gripper2/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def gripper3_on():
    # Wait till the srv is available
    rospy.wait_for_service('/urdF/vacuum_gripper3/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/urdF/vacuum_gripper3/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def gripper4_on():
    # Wait till the srv is available
    rospy.wait_for_service('/urdF/vacuum_gripper4/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/urdF/vacuum_gripper4/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def gripper5_on():
    # Wait till the srv is available
    rospy.wait_for_service('/urdF/vacuum_gripper5/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/urdF/vacuum_gripper5/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def gripper6_on():
    # Wait till the srv is available
    rospy.wait_for_service('/urdF/vacuum_gripper6/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/urdF/vacuum_gripper6/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)        

def gripper7_on():
    # Wait till the srv is available
    rospy.wait_for_service('/urdF/vacuum_gripper7/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/urdF/vacuum_gripper7/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

rospy.init_node("gripper_cmd", anonymous=False)

gripper_status_sub = rospy.Subscriber('/urdF/vacuum_gripper/grasping', Bool, gripper_status, queue_size=1)
gripper_status_sub = rospy.Subscriber('/urdF/vacuum_gripper1/grasping', Bool, gripper_status, queue_size=1)
gripper_status_sub = rospy.Subscriber('/urdF/vacuum_gripper2/grasping', Bool, gripper_status, queue_size=1)
gripper_status_sub = rospy.Subscriber('/urdF/vacuum_gripper3/grasping', Bool, gripper_status, queue_size=1)
gripper_status_sub = rospy.Subscriber('/urdF/vacuum_gripper4/grasping', Bool, gripper_status, queue_size=1)
gripper_status_sub = rospy.Subscriber('/urdF/vacuum_gripper5/grasping', Bool, gripper_status, queue_size=1)
gripper_status_sub = rospy.Subscriber('/urdF/vacuum_gripper6/grasping', Bool, gripper_status, queue_size=1)
gripper_status_sub = rospy.Subscriber('/urdF/vacuum_gripper7/grasping', Bool, gripper_status, queue_size=1)


gripper_on()
gripper1_on()
gripper2_on()
gripper3_on()
gripper4_on()
gripper5_on()
gripper6_on()
gripper7_on()

rospy.spin()
