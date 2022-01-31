#!/usr/bin/env python
import rospy
from robot_system.msg import Gui_trig
import os
from urllib import unquote
from subprocess import Popen
import time



def pose_save_gui_clbk(data):
    pose_saver_trig = 1
    
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




def pose_saver_gui():
    rospy.init_node("pose_saver_gui",anonymous =  True)
    rospy.Subscriber("/home_save_trig_gui", Gui_trig, pose_save_gui_clbk)
    print("DDDDD")
    rospy.spin()

if __name__ == "__main__":
    pose_saver_gui()
