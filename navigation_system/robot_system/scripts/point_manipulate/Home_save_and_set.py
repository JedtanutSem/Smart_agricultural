#!/usr/bin/env python

"""
TF from map and base_link
"""
import time
import rospy
import json
import tf
import os
import pandas as pd
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_system.msg import Robotpose
from std_msgs.msg import String
pose_robot = [0,0,0,0]
home_pose_read = [0.,0.,0.]



def home_map_tf_send(x,y,w):
    br = tf.TransformBroadcaster()
    #quatern_list = tf.transformations.quaternion_from_euler(0,0,w)
    br.sendTransform((x, y, 0),tf.transformations.quaternion_from_euler(0,0,w),rospy.Time.now(),"home","map")


def amcl_init_pose(x,y,w):
    initpose = PoseWithCovarianceStamped()
    initpose.header.stamp = rospy.get_rostime()
    initpose.header.frame_id = "map"
    initpose.pose.pose.position.x = x
    initpose.pose.pose.position.y = y
    quatern_list = tf.transformations.quaternion_from_euler(0,0,w)

    initpose.pose.pose.orientation.x = quatern_list[0]
    initpose.pose.pose.orientation.y = quatern_list[1]
    initpose.pose.pose.orientation.z = quatern_list[2]
    initpose.pose.pose.orientation.w = quatern_list[3]
    pub_init_pose.publish(initpose)
    print("publish!!")

def read_home_log():
    global home_pose_read
    path = rospy.get_param("Home_log_dir")
    home_log_df = pd.read_csv(path)
    home_log_dict = home_log_df.to_dict('dict')
    home_pose_read = [home_log_dict["x"][0],home_log_dict["y"][0],home_log_dict["w"][0]]
    print(home_pose_read)

def set_home_trig_clbk(msg):
    read_home_log()
    amcl_init_pose(home_pose_read[0],home_pose_read[1],home_pose_read[2])

def save_home_clbk(msg):
    try:
        home_set_str = msg.data
        json_acceptable_string = home_set_str.replace("'", "\"")
        home_set = json.loads(json_acceptable_string)
    except:
        print("not_save_from CiRA CORE")

    #path = "~/agri_ws/src/navigation_system/robot_system/log/homePose.csv"
    home_save_dir = rospy.get_param("Home_log_dir")
    print(home_save_dir)
    x_pose_current = pose_robot[0]
    y_pose_current = pose_robot[1]
    w_pose_current = pose_robot[2]
    home_pose_dict = {"x":[x_pose_current],"y":[y_pose_current],"w":[w_pose_current]}
    home_pose_dataframe = pd.DataFrame(home_pose_dict, columns= ['x','y','w'])
    home_pose_dataframe.to_csv(home_save_dir, header=True,index=True)
    amcl_init_pose(x_pose_current,y_pose_current,w_pose_current)
    read_home_log()
    home_map_tf_send(home_pose_read[0],home_pose_read[1],home_pose_read[2])

def init_start():
    read_home_log()
    time.sleep(2)
    amcl_init_pose(home_pose_read[0],home_pose_read[1],home_pose_read[2])
    home_map_tf_send(home_pose_read[0],home_pose_read[1],home_pose_read[2])

    print("init_start!!!!!")

def robot_map_tf_clbk(msg):
    global pose_robot
    pose_robot =  [msg.x_pose, msg.y_pose, msg.w_pose]
    #print(pose_robot)

def home_initial():
    global pub_init_pose
    rospy.init_node("home_initial", anonymous = True)
    rospy.Subscriber("/save_home_trig", String, save_home_clbk)
    rospy.Subscriber("/robot/position", Robotpose, robot_map_tf_clbk)
    rospy.Subscriber("/set_home_trig", String, set_home_trig_clbk)
    pub_init_pose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size = 10)
    init_start()

    #base_link_in = rospy.get_param("base_link")

    rate = rospy.Rate(1)
    #rospy.spin()
    while not rospy.is_shutdown():
        try:
            read_home_log()
            br = tf.TransformBroadcaster()
            #quatern_list = tf.transformations.quaternion_from_euler(0,0,w)
            br.sendTransform((home_pose_read[0], home_pose_read[1], 0),tf.transformations.quaternion_from_euler(0,0,home_pose_read[2]),rospy.Time.now(),"home","map")
        except Exception as e: print(e)
        rate.sleep()


if __name__ == "__main__":
    home_initial()
