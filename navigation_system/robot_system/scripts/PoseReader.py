#!/usr/bin/env python


import rospy
import json
import pandas as pd
from robot_system.msg import Robotpose
from std_msgs.msg import String

def PoseGoal_CLBK(data):
    data_str = data.data
    json_acceptable_string = data_str.replace("'", "\"")
    data_dict = json.loads(json_acceptable_string)
    pose_count = data_dict["poseCount"]

    poseCount = pose_count
    path = "~/agri_ws/src/navigation_system/robot_system/log/homePose_multi.csv"

    Pose_df = pd.read_csv('~/agri_ws/src/navigation_system/robot_system/log/poseSaver.csv')
    Pose_df.set_index('count', inplace = True)

    data = Pose_df

    pose_list = Pose_df.loc[poseCount,:]


    #print(pose_list['x'])

    #print(poseCount)


     #payload["x_pose"] = pose_list['x']
     #payload["y_pose"] = pose_list['y']
     #payload["w_pose"] = pose_list['w']


    msg = Robotpose()
    msg.x_pose = pose_list['x']
    msg.y_pose = pose_list['y']
    msg.w_pose = pose_list['w']
    #rospy.loginfo(msg)
    pub.publish(msg)


def PoseRead():
    global pub
    rospy.init_node("PoseReader", anonymous = True)
    rospy.loginfo("Star Position log reader node")
    rospy.Subscriber("pose_goal_json", String, PoseGoal_CLBK)
    pub = rospy.Publisher("pose_goal_from_log", Robotpose, queue_size=10)
    rospy.spin()

if __name__ == "__main__":
    PoseRead()
