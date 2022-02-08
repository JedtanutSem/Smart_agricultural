#!/usr/bin/env python

import rospy
import tf
import json
from std_msgs.msg import String
from robot_system.msg import Robotpose
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler



def cira_pose_clbk(msg):
    global position
    goal_str = msg.data
    pose = Robotpose()
    while not (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

        (trans,rot) = listener.lookupTransform('/map','/point',rospy.Time(0))
        rospy.loginfo(trans)
    pose.x_trans = trans[0]
    pose.y_trans = trans[1]
    pose.z_trans = trans[2]

    pose.x_rot = rot[0]
    pose.y_rot = rot[1]
    pose.z_rot = rot[2]
    pose.w_rot = rot[3]
    pub.publish(pose)
    rospy.loginfo(pose)

def cira_pose_listen():
    global pub
    rospy.init_node('pose_pub_gui')
    rospy.Subscriber('/pose_cira',String, cira_pose_clbk)
    pub = rospy.Publisher('/tf_map_home',Robotpose,queue_size = 10)
    rospy.spin()
if __name__ == '__main__':
    cira_pose_listen()

#rospy.loginfo(msg)
