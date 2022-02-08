#!/usr/bin/env python
import rospy
import os
import pandas as pd

from std_msgs.msg import String
from robot_system.msg import Robotpose


def poseSaver_reset():
    path = rospy.get_param('pose_list_dir')
    os.remove(path)
    x = []
    y = []
    w = []
    count =[]
    homePose_multi_dict = {"count":count,"x":x, "y":y, "w":w}
    homePose = pd.DataFrame(homePose_multi_dict, columns= ['count','x','y','w'])
    homePose.set_index('count', inplace = True)
    homePose.to_csv(path, header=True,index=True)
    msg = Robotpose()
    msg.state = "File reset!!!"
    #pub.publish(msg)
    rospy.loginfo("File reset!!!")
    x = [0.0000]
    y = [0.0000]
    w = [0.0000]
    for count in range(32):
        #count = homePose_dict_get['count']
        homePose_multi_dict = {"count":count,"x":x, "y":y, "w":w}
        #rospy.loginfo(homePose_multi_dict)
        homePose = pd.DataFrame(homePose_multi_dict, columns= ['count','x','y','w'])
        homePose.set_index('count', inplace = True)
        if(count == 1):
            homePose.to_csv('~/agri_ws/src/navigation_system/robot_system/log/poseSaver.csv',mode='a', header=False,index=True)
        elif(count>0):
            homePose.to_csv('~/agri_ws/src/navigation_system/robot_system/log/poseSaver.csv',mode='a', header=False,index=True)

def RTlog_reset():
    robot_position = [0.,0.,0.]
    RTlog_save_dir = rospy.get_param("RTlog_save_dir")
    RTlog_pose_dict = {"x":[robot_position[0]],"y":[robot_position[1]],"w":[robot_position[2]]}
    RTlog_pose_dataframe = pd.DataFrame(RTlog_pose_dict, columns= ['x','y','w'])
    RTlog_pose_dataframe.to_csv(RTlog_save_dir, header=True,index=True)

def factory_reset_clbk(msg):
    poseSaver_reset()
    RTlog_reset()

def factory_reset():
    rospy.init_node("factory_reset")
    rospy.Subscriber("/factory_reset_trig",String,factory_reset_clbk)
    rospy.spin()

if __name__ == "__main__":
    factory_reset()
