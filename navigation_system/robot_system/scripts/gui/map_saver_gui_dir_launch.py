#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import os
from urllib import unquote
from subprocess import Popen
import time
from Tkinter import *
import Tkinter as tk
import tkFileDialog
#import ttk
import tkFont


def map_saver_dir_clbk(msg):
    #msg = msg.data

    msg_state = String()
    msg_state.data = "Saving"
    pub_save_finished.publish(msg_state)
    dir_str = msg.data
    print("NAME:  "+ str())
    print('###########  Smart_Agricultural Autorun ############')
    import getpass
    username = getpass.getuser()
    print('user : ', username)

    Popen(["gnome-terminal", '-x' , 'bash', '-c' ,'source /opt/ros/melodic/setup.bash && roscore'])
    time.sleep(3)
    cmd_save = 'source /opt/ros/melodic/setup.bash && source ~/agri_ws/devel/setup.bash && rosrun map_server map_saver -f ' + str(dir_str)
    print(cmd_save)
    Popen(["gnome-terminal", '-x' , 'bash', '-c' ,cmd_save])
    msg_state.data = "Finished"
    pub_save_finished.publish(msg_state)
    print("Finished")




def map_saver_dir():
    global pub_save_finished
    rospy.init_node("map_saver_gui_dir_launch", anonymous = True)
    rospy.Subscriber("/map_saver_dir",String,map_saver_dir_clbk)
    pub_save_finished = rospy.Publisher("/save_map_state", String, queue_size = 10)
    rospy.spin()


if __name__ == "__main__":
    map_saver_dir()
