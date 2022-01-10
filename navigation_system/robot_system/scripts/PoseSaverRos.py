#!/usr/bin/env python


import rospy
import json
import os
import pandas as pd
from std_msgs.msg import String
from robot_system.msg import Robotpose
x_pose = 0
y_pose = 0
w_pose = 0


def Robotpose_CLBK(msg):
    global x_pose
    global y_pose
    global w_pose
    x_pose = msg.x_pose
    y_pose = msg.y_pose
    w_pose = msg.w_pose
    #rospy.loginfo(data)

def PoseSaver_CLBK(data):
    data_str = data.data
    json_acceptable_string = data_str.replace("'", "\"")
    data_dict = json.loads(json_acceptable_string)
    pose_count = data_dict["poseCount"]
    pose_saver_trig = int(data_dict["PoseSaverTrig"])
    if(pose_saver_trig == 1):
        #rospy.loginfo(pose_count)
        path = "~/agri_ws/src/navigation_system/robot_system/log/poseSaver.csv"
        poseCount = pose_count
        x_pose_current = x_pose
        y_pose_current = y_pose
        w_pose_current = w_pose

        homePose_df = pd.read_csv(path)
        #print(type(poseCount))
        homePose_df.set_index('count', inplace = True)
        #data = homePose_df.drop(1)
        homePose_df.at[poseCount,'x'] = x_pose_current #input the row data and value
        homePose_df.at[poseCount,'y'] =y_pose_current
        homePose_df.at[poseCount,'w'] = w_pose_current
        data = homePose_df
        print(data)
        data.to_csv(path, header=True,index=True)
        msg = Robotpose()
        msg.state = "Position NO: " +str(poseCount)+" Saved!!!"
        pub.publish(msg)
        rospy.loginfo("Position NO: " +str(poseCount)+" Saved!!!")



def PoseReset_CLBK(data):
    data_str = data.data
    json_acceptable_string = data_str.replace("'", "\"")
    data_dict = json.loads(json_acceptable_string)
    pose_reset_trig = int(data_dict["PoseResetTrig"])
    if(pose_reset_trig == 1):
        os.remove("/home/agri/agri_ws/src/navigation_system/robot_system/log/poseSaver.csv")
        x = []
        y = []
        w = []
        count =[]
        homePose_multi_dict = {"count":count,"x":x, "y":y, "w":w}
        homePose = pd.DataFrame(homePose_multi_dict, columns= ['count','x','y','w'])
        homePose.set_index('count', inplace = True)
        homePose.to_csv('~/agri_ws/src/navigation_system/robot_system/log/poseSaver.csv', header=True,index=True)
        msg = Robotpose()
        msg.state = "File reset!!!"
        pub.publish(msg)
        rospy.loginfo("File reset!!!")
        x = [0.0000]
        y = [0.0000]
        w = [0.0000]
        for count in range(32):
            #count = homePose_dict_get['count']
            homePose_multi_dict = {"count":count,"x":x, "y":y, "w":w}
            rospy.loginfo(homePose_multi_dict)
            homePose = pd.DataFrame(homePose_multi_dict, columns= ['count','x','y','w'])
            homePose.set_index('count', inplace = True)
            if(count == 1):
                homePose.to_csv('~/agri_ws/src/navigation_system/robot_system/log/poseSaver.csv',mode='a', header=False,index=True)
            elif(count>0):
                homePose.to_csv('~/agri_ws/src/navigation_system/robot_system/log/poseSaver.csv',mode='a', header=False,index=True)


def PoseSaveListen_Trig():
    global pub
    rospy.init_node("PoseSaveRos_Trig",anonymous = True)
    rospy.loginfo("Start position log saver node")
    rospy.Subscriber("/posesaver_trig", String, PoseSaver_CLBK) #Save postion to csv log file
    rospy.Subscriber("/robot/position", Robotpose, Robotpose_CLBK) #Subscribe current robot position
    rospy.Subscriber("/posereset_trig", String, PoseReset_CLBK) #Reset csv log file
    pub = rospy.Publisher("/homePoseSavedData", Robotpose,queue_size = 10)
    rospy.spin()

if __name__ == "__main__":
    PoseSaveListen_Trig()
