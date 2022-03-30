#!/usr/bin/env python

from robot_msg.msg import Pose_RobotPose

home_log_dir_default = '/home/jedtanut/agri_ws/src/navigation_system/robot_system/log/homeSave.csv'
home_log_dir = rospy.get_param('Home_log_dir',home_log_dir_default)

class Node:
    def __init__(self,node_name,rate):
        self.node_name = node_name
        self.rate      = rate
        rospy.init_node(self.node_name, anonymous=True)
        rospy.Subsciber('pose_robot_tf',Pose_RobotPose,self.pose_robot_tf_clbk)

    def pose_robot_tf_clbk(self,data):
        self.x_euler    = data.x_euler
        self.y_euler    = data.y_euler
        self.yaw_euler  = data.yaw_euler

    def save_pose(self,zone,dir):
        home_pose_dict  = {'x'   :[self.x_euler],
                           'y'   :[self.y_euler],
                           'yaw' :[self.yaw_euler]}
        home_pose_df    = pd.DataFrame(home_pose_dict, columns=['x','y','yaw'])
        home_pose_df.to_csv(dir,header=True,index=True)
