#!/usr/bin/env python

"""
TF from map and base_link
"""
import rospy
import json
import tf
import os
import pandas as pd
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
pose_amcl = [0,0,0,0]

def home_pose_init_clbk(msg):
    try:
        home_set_str = msg.data
        json_acceptable_string = home_set_str.replace("'", "\"")
        home_set = json.loads(json_acceptable_string)
    except:
        print("not_save_from CiRA CORE")

    #path = "~/agri_ws/src/navigation_system/robot_system/log/homePose.csv"
    home_save_dir = rospy.get_param("home_save_dir")
    print(home_save_dir)
    x_pose_current = pose_amcl[0]
    y_pose_current = pose_amcl[1]
    w_pose_current = pose_amcl[2]
    home_pose_dict = {"x":[x_pose_current],"y":[y_pose_current],"w":[w_pose_current]}
    home_pose_dataframe = pd.DataFrame(home_pose_dict, columns= ['x','y','w'])
    home_pose_dataframe.to_csv(home_save_dir, header=True,index=True)




def pose_tf_map_point_clbk(msg):
    global pose_amcl
    pose_msg = msg
    position = pose_msg.pose.pose.position
    x_pose = position.x
    y_pose = position.y
    z_pose = position.z

    orientation = pose_msg.pose.pose.orientation
    x_orien = orientation.x
    y_orien = orientation.y
    z_orien = orientation.z
    w_orien = orientation.w
    rot = (x_orien, y_orien, z_orien, w_orien)
    (roll, pitch, yaw) = euler_from_quaternion(rot)
    pose_amcl =  [x_pose, y_pose, yaw]
    print(pose_amcl)

def home_initial():
    rospy.init_node("home_initial", anonymous = True)
    rospy.Subscriber("/home_pose_trig", String, home_pose_init_clbk)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, pose_tf_map_point_clbk)
    #base_link_in = rospy.get_param("base_link")


    rospy.spin()

if __name__ == "__main__":
    home_initial()
