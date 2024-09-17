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

def gripper_off():
    rospy.wait_for_service('/urdF/vacuum_gripper/off')
    try:
        turn_off = rospy.ServiceProxy('/urdF/vacuum_gripper/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def gripper1_off():
    rospy.wait_for_service('/urdF/vacuum_gripper1/off')
    try:
        turn_off = rospy.ServiceProxy('/urdF/vacuum_gripper1/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def gripper2_off():
    rospy.wait_for_service('/urdF/vacuum_gripper2/off')
    try:
        turn_off = rospy.ServiceProxy('/urdF/vacuum_gripper2/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def gripper3_off():
    rospy.wait_for_service('/urdF/vacuum_gripper3/off')
    try:
        turn_off = rospy.ServiceProxy('/urdF/vacuum_gripper3/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
     

def gripper4_off():
    rospy.wait_for_service('/urdF/vacuum_gripper4/off')
    try:
        turn_off = rospy.ServiceProxy('/urdF/vacuum_gripper4/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def gripper5_off():
    rospy.wait_for_service('/urdF/vacuum_gripper5/off')
    try:
        turn_off = rospy.ServiceProxy('/urdF/vacuum_gripper5/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def gripper6_off():
    rospy.wait_for_service('/urdF/vacuum_gripper6/off')
    try:
        turn_off = rospy.ServiceProxy('/urdF/vacuum_gripper6/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def gripper7_off():
    rospy.wait_for_service('/urdF/vacuum_gripper7/off')
    try:
        turn_off = rospy.ServiceProxy('/urdF/vacuum_gripper7/off', Empty)
        resp = turn_off()
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

gripper_off()
gripper1_off()
gripper2_off()
gripper3_off()
gripper4_off()
gripper5_off()
gripper6_off()
gripper7_off()

rospy.spin()
