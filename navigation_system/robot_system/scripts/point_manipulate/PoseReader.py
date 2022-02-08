#!/usr/bin/env python

"""
For read log position from csv file (Cira core must send a number of position that save log)
"""
import rospy
import json
import pandas as pd
from robot_system.msg import Robotpose
from std_msgs.msg import String

def PoseRead_CLBK():


    path = rospy.get_param("pose_list_dir")
    Pose_df = pd.read_csv(path)
    Pose_df.set_index('count', inplace = True)
    data = Pose_df
    #pose_list = Pose_df.loc[total_pose_list,:]
    print(data)

    #pub.publish(msg)


def PoseRead():
    global pub
    rospy.init_node("PoseReader", anonymous = True)
    rospy.loginfo("Start Position log reader node")
    rospy.Subscriber("pose_goal_json", String, PoseRead_CLBK)
    PoseRead_CLBK()
    pub = rospy.Publisher("pose_goal_from_log", Robotpose, queue_size=10)
    rospy.spin()

if __name__ == "__main__":
    PoseRead()
