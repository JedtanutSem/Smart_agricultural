#!/usr/bin/env python

from Tkinter import *
import Tkinter as tk
import tkFileDialog
#import ttk
import tkFont
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from robot_system.msg import Gui_trig, PID_Gui
from std_msgs.msg import String
import time
import pandas
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
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

        self.GLabel_857=tk.Label(root)
        self.ft = tkFont.Font(family='Times',size=23)
        self.GLabel_857["font"] = ft
        self.GLabel_857["fg"] = "#333333"
        self.GLabel_857["justify"] = "center"
        self.GLabel_857["text"] = "Smart Agricultural Robot System"
        self.GLabel_857.place(x=70,y=40,width=513,height=30)


        self.Slider_Bar_P = tk.Scale(root)
        self.Slider_Bar_P["from"] = 0
        self.Slider_Bar_P["to"] = 10
        self.Slider_Bar_P.place(x=50,y=300,width=100,height=80)
        self.Slider_Bar_P["orient"] = HORIZONTAL
        self.Slider_Bar_P["label"] = "P val"
        self.Slider_Bar_P["showvalue"] = True
        self.Slider_Bar_P["resolution"] = 0.5
        self.Slider_Bar_P["command"] = self.slider_P_changed
        #self.Slider_Bar.set(rospy.get_param("p_value"))
        #self.Slider_Bar["cursor#"] = 0.5
        #w1 = Scale(root, from_=0, to=42)
        #w1.pack()
        #GLabel_857["text"] = self.test


        self.Slider_Bar_I = tk.Scale(root)
        self.Slider_Bar_I["from"] = 0
        self.Slider_Bar_I["to"] = 10
        self.Slider_Bar_I.place(x=250,y=300,width=100,height=80)
        self.Slider_Bar_I["orient"] = HORIZONTAL
        self.Slider_Bar_I["label"] = "I val"
        self.Slider_Bar_I["showvalue"] = True
        self.Slider_Bar_I["resolution"] = 0.5
        self.Slider_Bar_I["command"] = self.slider_I_changed

        self.Slider_Bar_D = tk.Scale(root)
        self.Slider_Bar_D["from"] = 0
        self.Slider_Bar_D["to"] = 10
        self.Slider_Bar_D.place(x=450,y=300,width=100,height=80)
        self.Slider_Bar_D["orient"] = HORIZONTAL
        self.Slider_Bar_D["label"] = "D val"
        self.Slider_Bar_D["showvalue"] = True
        self.Slider_Bar_D["resolution"] = 0.5
        self.Slider_Bar_D["command"] = self.slider_D_changed





        self.SetPID_BTN=tk.Button(root)
        self.SetPID_BTN["activeforeground"] = "#6cb8f2"
        self.SetPID_BTN["bg"] = "#ffffff"
        self.ft = tkFont.Font(family='Times',size=13)
        self.SetPID_BTN["font"] = ft
        self.SetPID_BTN["fg"] = "#000000"
        self.SetPID_BTN["justify"] = "center"
        self.SetPID_BTN["text"] = "Set PID Val"
        self.SetPID_BTN.place(x=300,y=140,width=163,height=41)
        self.SetPID_BTN["command"] = self.SetPID_BTN_command

        self.Browse_PATH=tk.Button(root)
        self.Browse_PATH["activeforeground"] = "#6cb8f2"
        self.Browse_PATH["bg"] = "#ffffff"
        self.ft = tkFont.Font(family='Times',size=13)
        self.Browse_PATH["font"] = ft
        self.Browse_PATH["fg"] = "#000000"
        self.Browse_PATH["justify"] = "center"
        self.Browse_PATH["text"] = "Browse"
        self.Browse_PATH.place(x=500,y=140,width=163,height=41)
        self.Browse_PATH["command"] = self.Browse_PATH_command


        self.Label_PATH=tk.Label(root)
        self.ft = tkFont.Font(family='Times',size=23)
        self.Label_PATH["font"] = ft
        self.Label_PATH["fg"] = "#333333"
        self.Label_PATH["justify"] = "center"
        self.Label_PATH["text"] = "No PATH Selected"
        self.Label_PATH.place(x=200,y=200,width=513,height=30)

        self.Text_INPUT = tk.Entry(root)
        #self.ft = tkFont.Font(family='Times',size=23)
        #self.Text_INPUT["text"] = "Print"
        #self.Text_INPUT["command"] = self.Text_INPUT_command
        self.Text_INPUT.place(x=200,y=250,width=100,height=30)

        self.SaveHome=tk.Button(root)
        self.SaveHome["activeforeground"] = "#6cb8f2"
        self.SaveHome["bg"] = "#ffffff"
        self.ft = tkFont.Font(family='Times',size=13)
        self.SaveHome["font"] = ft
        self.SaveHome["fg"] = "#000000"
        self.SaveHome["justify"] = "center"
        self.SaveHome["text"] = "SaveHome"
        self.SaveHome.place(x=300,y=250,width=163,height=41)
        self.SaveHome["command"] = self.SaveHome_command

        self.Fac_reset=tk.Button(root)
        self.Fac_reset["activeforeground"] = "#6cb8f2"
        self.Fac_reset["bg"] = "#ffffff"
        self.ft = tkFont.Font(family='Times',size=13)
        self.Fac_reset["font"] = ft
        self.Fac_reset["fg"] = "#000000"
        self.Fac_reset["justify"] = "center"
        self.Fac_reset["text"] = "Fac RST"
        self.Fac_reset.place(x=300,y=350,width=163,height=41)
        self.Fac_reset["command"] = self.Fac_reset_command

        self.SetHome=tk.Button(root)
        self.SetHome["activeforeground"] = "#6cb8f2"
        self.SetHome["bg"] = "#ffffff"
        self.ft = tkFont.Font(family='Times',size=13)
        self.SetHome["font"] = ft
        self.SetHome["fg"] = "#000000"
        self.SetHome["justify"] = "center"
        self.SetHome["text"] = "Set Home"
        self.SetHome.place(x=300,y=500,width=163,height=41)
        self.SetHome["command"] = self.SetHome_command




        GButton_254=tk.Button(root)
        GButton_254["bg"] = "#eb5b5b"
        ft = tkFont.Font(family='Times',size=18)
        GButton_254["font"] = ft
        GButton_254["fg"] = "#ffffff"
        GButton_254["justify"] = "center"
        GButton_254["text"] = "Exit"
        GButton_254.place(x=int(self.Slider_Bar_P.get()),y=660,width=100,height=39)
        GButton_254["command"] = self.GButton_254_command

        self.listen()

        """

        img = cv2.imread("/home/agri/Desktop/Screenshot from 2022-02-01 11-38-38.png")

        print("Shape of the loaded image is", img.shape)
        shape = img.shape
        imgRGB = img[:,:,::-1]
        imgtk = Image.fromarray(imgRGB)
        dd = tk.Label(root)
        dd['image'] = imgtk
        dd.place(x=550,y=660,width=100,height=39)


        cv2.imshow("PIC",img)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            root.destroy()

        """





    def vel_clbk(self,data):
        msg = data
        print(msg)
        x = msg.linear.x
        #self.GLabel_857["text"] = x

    def save_map_state_clbk(self,data):
        state_str = data.data
        if(state_str == "Finished"):
            log_state = "save leaww : " + str(self.dir_save)
            self.Label_PATH["text"] = log_state

            time.sleep(3)
            log_state = "Please select path"
            self.Label_PATH["text"] = log_state
            print("save leaww")
        else:
            print("Saving")


    def listen(self):
        rospy.Subscriber("/cmd_vel", Twist, self.vel_clbk)
        rospy.Subscriber("/save_map_state",String, self.save_map_state_clbk)
        self.pub_home_save_trig = rospy.Publisher("/save_home_trig",String,queue_size = 10)
        self.pub_factory_reset_trig = rospy.Publisher("factory_reset_trig",String,queue_size = 10)
        self.pub_set_home_trig = rospy.Publisher("set_home_trig",String,queue_size = 10)

    def Fac_reset_command(self):
        msg = String()
        msg.data="1"
        self.pub_factory_reset_trig.publish(msg)

    def SetHome_command(self):
        msg = String()
        msg.data = "1"
        self.pub_set_home_trig.publish(msg)



    def Text_INPUT_command(self):
        inp = self.Text_INPUT.get()
        print(inp)

    def SaveHome_command(self):
        msg=String()
        msg.data="Test"
        print(msg)
        self.pub_home_save_trig.publish(msg)


    def Browse_PATH_command(self):
        name = self.Text_INPUT.get()
        len_name_file = len(name)

        if(len_name_file == 0):


            print("Please fill name")
            sub_fill_name = tk.Tk()
            sub_fill_name.title("Caution")
            width=364
            height=186
            screenwidth = sub_fill_name.winfo_screenwidth()
            screenheight = sub_fill_name.winfo_screenheight()
            alignstr = '%dx%d+%d+%d' % (width, height, (screenwidth - width) / 2, (screenheight - height) / 2)
            sub_fill_name.geometry(alignstr)
            sub_fill_name.resizable(width=False, height=False)


            Label_name_fill_caution=tk.Label(sub_fill_name)
            ft = tkFont.Font(family='Times',size=18)
            Label_name_fill_caution["font"] = ft
            Label_name_fill_caution["fg"] = "#cc0000"
            Label_name_fill_caution["justify"] = "center"
            Label_name_fill_caution["text"] = "Sai chue duay"
            Label_name_fill_caution.place(x=10,y=40,width=351,height=30)

        elif(len_name_file > 0):
            fname = tkFileDialog.askdirectory(initialdir="/home/agri/agri_ws/src/Smart_agricultural/navigation_system/robot_system/map")
            #self.dir_str = str(fname)

            path = "~/agri_ws/src/navigation_system/robot_system/log/poseSaver.csv"
            #DataFrame_MAP_Read = pd.read_csv(path)
            #.to_csv('~/agri_ws/src/navigation_system/robot_system/log/poseSaver.csv',mode='a', header=False,index=True)

            fname = fname+'/'+name+'.yaml'
            self.Label_PATH["text"] = fname
            msg = String()
            msg.data = fname
            print(fname)
            pub_map_saver_dir.publish(msg)
            self.dir_save = fname


        #print fname
    def slider_P_changed(self,event):
        #print(self.Slider_Bar_P.get())
        P_val = float(self.Slider_Bar_P.get())
        I_val = float(self.Slider_Bar_I.get())
        D_val = float(self.Slider_Bar_D.get())
        msg = PID_Gui()
        msg.P = P_val
        msg.I = I_val
        msg.D = D_val
        pub_PID.publish(msg)
        root.update()

    def slider_I_changed(self,event):
        #print(self.Slider_Bar_I.get())
        P_val = float(self.Slider_Bar_P.get())
        I_val = float(self.Slider_Bar_I.get())
        D_val = float(self.Slider_Bar_D.get())
        msg = PID_Gui()
        msg.P = P_val
        msg.I = I_val
        msg.D = D_val
        pub_PID.publish(msg)


    def slider_D_changed(self,event):
        #print(self.Slider_Bar_D.get())
        P_val = float(self.Slider_Bar_P.get())
        I_val = float(self.Slider_Bar_I.get())
        D_val = float(self.Slider_Bar_D.get())
        msg = PID_Gui()
        msg.P = P_val
        msg.I = I_val
        msg.D = D_val
        pub_PID.publish(msg)

    def SetPID_BTN_command(self):
        #print(self.Slider_Bar_I.get())

        #global P_val, I_val, D_val
        P_val = float(self.Slider_Bar_P.get())
        I_val = float(self.Slider_Bar_I.get())
        D_val = float(self.Slider_Bar_D.get())

        msg = PID_Gui()
        msg.P = P_val
        msg.I = I_val
        msg.D = D_val

        #pub_PID.publish(msg)



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
            #msg = Gui_trig()
            #msg.State_trig = 0
            #pub_home_save_trig.publish(msg)
            #GLabel_857=tk.Label(root)
            #self.GLabel_857["text"]="NO"
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




def callback(data):
  try:
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
  except CvBridgeError as e:
    print(e)
  cv2.imshow("Image window", cv_image)

def save_map_state_clbk(data):
    state_str = data.data
    if(state_str == "Finished"):
        save_state_window = tk.Tk()
        save_state_window.title("Caution")
        width=364
        height=186
        screenwidth = save_state_window.winfo_screenwidth()
        screenheight = save_state_window.winfo_screenheight()
        alignstr = '%dx%d+%d+%d' % (width, height, (screenwidth - width) / 2, (screenheight - height) / 2)
        save_state_window.geometry(alignstr)
        save_state_window.resizable(width=False, height=False)

        GLabel_915=tk.Label(save_state_window)
        #ft = tkFont.Font(family='Times',size=18)
        #GLabel_915["font"] = ft
        GLabel_915["fg"] = "#cc0000"
        GLabel_915["justify"] = "center"
        GLabel_915["text"] = "Confirm to Save Home Position??"
        GLabel_915.place(x=10,y=40,width=351,height=30)
    else:
        print("Saving")



if __name__ == "__main__":
    global img,bridge
    rospy.init_node("gui_node", anonymous = True)
    #image_sub = rospy.Subscriber("/camera/usb_cam/image_raw",Image,callback)
    #bridge = CvBridge()
    pub_home_save_trig = rospy.Publisher("/home_save_trig_gui",Gui_trig,queue_size = 10)
    pub_map_saver_dir = rospy.Publisher("/map_saver_dir",String,queue_size = 10)
    #rospy.Subscriber("/cmd_vel", Twist, vel_clbk)
    pub_PID = rospy.Publisher("/PID_Gui", PID_Gui, queue_size = 10)
    print("Loop")
    root = tk.Tk()
    app = App(root)
    rate = rospy.Rate(10)
    #

    root.update()
    #rospy.spin()
        #cv2.startWindowThread()




    root.mainloop()
