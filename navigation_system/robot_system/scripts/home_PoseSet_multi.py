#!/usr/bin/env python
import xlsxwriter
import pandas as pd
from xlsxwriter import Workbook
import rospy
import json
import os
from std_msgs.msg import String
from robot_system.msg import Robotpose

def poseCLBK(data):
    homePose_str = data.data
    json_acceptable_string = homePose_str.replace("'", "\"")
    homePose_dict_get = json.loads(json_acceptable_string)

    #rospy.loginfo(homePose_dict_get)
    """
    x = homePose_dict_get['x_pose']
    y = homePose_dict_get['y_pose']
    w = homePose_dict_get['w_pose']
    x = [0]
    y = [0]
    w = [0]
    """
    x = [0]
    y = [0]
    w = [0]
    for count in range(32):
        #count = homePose_dict_get['count']
        homePose_multi_dict = {"count":count,"x":x, "y":y, "w":w}
        rospy.loginfo(homePose_multi_dict)
        homePose = pd.DataFrame(homePose_multi_dict, columns= ['count','x','y','w'])
        homePose.set_index('count', inplace = True)
        if(count == 1):
            homePose.to_csv('~/agri_ws/src/navigation_system/robot_system/log/homePose_multi.csv',mode='a', header=False,index=True)
        elif(count>0):
            homePose.to_csv('~/agri_ws/src/navigation_system/robot_system/log/homePose_multi.csv',mode='a', header=False,index=True)

def homePoseListen():
    global pub
    rospy.init_node('homePoseToCSV',anonymous = True)
    rospy.loginfo("Starting homePoseToCSV Node")
    rospy.Subscriber('/multihomePose', String, poseCLBK)
    pub = rospy.Publisher('/homePoseSavedData',Robotpose, queue_size=10)
    rospy.spin()

if __name__ == "__main__":
    homePoseListen()
