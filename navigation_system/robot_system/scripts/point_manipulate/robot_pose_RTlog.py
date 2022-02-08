#!/usr/bin/env python
import rospy

import os
import pandas as pd

from std_msgs.msg import String
from robot_system.msg import Robotpose
#robot_position = [,,]

def robot_map_tf_clbk(data):
    global robot_position
    x_pose = data.x_pose
    y_pose = data.y_pose
    w_pose = data.w_pose
    robot_position = [x_pose,y_pose,w_pose]

def robot_pose_RTSave():
    RTlog_save_dir = rospy.get_param("RTlog_save_dir")
    RTlog_pose_dict = {"x":[robot_position[0]],"y":[robot_position[1]],"w":[robot_position[2]]}
    RTlog_pose_dataframe = pd.DataFrame(RTlog_pose_dict, columns= ['x','y','w'])
    RTlog_pose_dataframe.to_csv(RTlog_save_dir, header=True,index=True)


def robot_pose_RTlog():
    rospy.init_node("robot_pose_RTlog", anonymous = True)
    rospy.Subscriber("robot/position",Robotpose,robot_map_tf_clbk)
    rospy.loginfo("Starting RTlog Node")
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            robot_pose_RTSave()
        except Exception as e: print(e)
        rate.sleep()


if __name__ == "__main__":
    robot_pose_RTlog()
