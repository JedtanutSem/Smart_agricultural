#!/usr/bin/env python
import rospy
import json

def Ai_json_clbk(msg):


def robot_ctrl_init():
    rospy.init_node("robot_ctrl_image", anonymous = True)
    rospy.Subscriber("/Ai_json", String, Ai_json_clbk)
