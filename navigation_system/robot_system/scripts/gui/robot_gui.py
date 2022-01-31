#!/usr/bin/env python

from Tkinter import *
import Tkinter as tk
#import ttk
import tkFont
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from robot_system.msg import Gui_trig
x_vel = 0

from PIL import Image, ImageTk

class App:
    def __init__(self, root):
        #setting title
        root.title("Smart Robot")
        #setting window size
        width=673
        height=724
        screenwidth = root.winfo_screenwidth()
        screenheight = root.winfo_screenheight()
        alignstr = '%dx%d+%d+%d' % (width, height, (screenwidth - width) / 2, (screenheight - height) / 2)
        root.geometry(alignstr)
        root.resizable(width=False, height=False)

        GButton_107=tk.Button(root)
        GButton_107["activeforeground"] = "#6cb8f2"
        GButton_107["bg"] = "#ffffff"
        ft = tkFont.Font(family='Times',size=13)
        GButton_107["font"] = ft
        GButton_107["fg"] = "#000000"
        GButton_107["justify"] = "center"
        GButton_107["text"] = "Save Home POSE"
        GButton_107.place(x=30,y=140,width=163,height=41)
        GButton_107["command"] = self.GButton_107_command

        GLabel_857=tk.Label(root)
        ft = tkFont.Font(family='Times',size=23)
        GLabel_857["font"] = ft
        GLabel_857["fg"] = "#333333"
        GLabel_857["justify"] = "center"
        #GLabel_857["text"] = "Smart Agricultural Robot System"

        GLabel_857.place(x=70,y=40,width=513,height=30)

        GButton_254=tk.Button(root)
        GButton_254["bg"] = "#eb5b5b"
        ft = tkFont.Font(family='Times',size=18)
        GButton_254["font"] = ft
        GButton_254["fg"] = "#ffffff"
        GButton_254["justify"] = "center"
        GButton_254["text"] = "Exit"
        GButton_254.place(x=550,y=660,width=100,height=39)
        GButton_254["command"] = self.GButton_254_command


        GListBox_92=tk.Listbox(root)
        GListBox_92["borderwidth"] = "1px"
        ft = tkFont.Font(family='Times',size=10)
        GListBox_92["font"] = ft
        GListBox_92["fg"] = "#333333"
        GListBox_92["justify"] = "center"
        GListBox_92.place(x=420,y=220,width=80,height=25)
        GListBox_92["exportselection"] = "0"
        GListBox_92["listvariable"] = "1,2,3,4,5,6,7,8,9,10"
        GListBox_92["selectmode"] = "single"
        #GListBox_92["setgrid"] = "True




    def GButton_107_command(self):
        #button1 = tk.Button(root, command=funA)
        #button1.pack()
        sub = tk.Tk()
        sub.title("Caution")
        width=364
        height=186
        screenwidth = sub.winfo_screenwidth()
        screenheight = sub.winfo_screenheight()
        alignstr = '%dx%d+%d+%d' % (width, height, (screenwidth - width) / 2, (screenheight - height) / 2)
        sub.geometry(alignstr)
        sub.resizable(width=False, height=False)
        def GButton_831_command():
            msg = Gui_trig()
            msg.State_trig = 0
            pub_home_save_trig.publish(msg)
            #print('subUINO')
            sub.destroy()
        def GButton_892_command():
            msg = Gui_trig()
            msg.State_trig = 1
            pub_home_save_trig.publish(msg)
            #print('subUIYES')



        GButton_892=tk.Button(sub)
        GButton_892["bg"] = "#3b3b3b"
        ft = tkFont.Font(family='Times',size=10)
        GButton_892["font"] = ft
        GButton_892["fg"] = "#ffffff"
        GButton_892["justify"] = "center"
        GButton_892["text"] = "Yes"
        GButton_892.place(x=90,y=150,width=70,height=25)
        GButton_892["command"] = GButton_892_command

        GButton_831=tk.Button(sub)
        GButton_831["bg"] = "#3b3b3b"
        ft = tkFont.Font(family='Times',size=10)
        GButton_831["font"] = ft
        GButton_831["fg"] = "#ffffff"
        GButton_831["justify"] = "center"
        GButton_831["text"] = "No"
        GButton_831.place(x=200,y=150,width=70,height=25)
        GButton_831["command"] = GButton_831_command

        GLabel_915=tk.Label(sub)
        ft = tkFont.Font(family='Times',size=18)
        GLabel_915["font"] = ft
        GLabel_915["fg"] = "#cc0000"
        GLabel_915["justify"] = "center"
        GLabel_915["text"] = "Confirm to Save Home Position??"
        GLabel_915.place(x=10,y=40,width=351,height=30)
    def GButton_254_command(self):
        print("Exit")
        root.destroy()

def vel_clbk(data):
    global x_vel
    x_vel = data.linear.x
    GLabel_857["text"] = x_vel

    print(x_vel)


if __name__ == "__main__":
    global img
    rospy.init_node("gui_node", anonymous = True)
    pub_home_save_trig = rospy.Publisher("/home_save_trig_gui",Gui_trig,queue_size = 10)
    rospy.Subscriber("/cmd_vel", Twist, vel_clbk)
    print("Loop")
    root = tk.Tk()
    app = App(root)
    rate = rospy.Rate(10)
    #rospy.spin()
    root.mainloop()
    rate.sleep()
