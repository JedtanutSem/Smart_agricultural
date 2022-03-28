#!/usr/bin/env python

import rospy
import os
import time

from realtime_pose import Pose_Realtime
from pose_recovery import Pose_recovery
from robot_pose_tf import Robot_pose_tf

from robot_msg.msg import State_RobotState, Pose_RobotPose

node_name           = 'Pose_RTsave'
rate                = rospy.get_param('rate_RT_Save', 2)

parent_frame        = rospy.get_param('parent_frame', '/map')
child_frame         = rospy.get_param('child_frame' , '/base_footprint')

file_path_recov     = '/home/jedtanut/agri_ws/src/navigation_system/robot_system/log/recov_state.csv'
file_path_RTlog     = '/home/jedtanut/agri_ws/src/navigation_system/robot_system/log/RTlog.csv.csv'

recov_path          = rospy.get_param('recov_path', file_path_recov)
RTlog_path          = rospy.get_param('RTlog_path', file_path_RTlog)

class RT_Pose:
    def __init__(self, parent_frame, child_frame, recov_path):
        self.pub = rospy.Publisher('/pose_recovery_state', State_RobotState, queue_size=10)
        time.sleep(1)
        self.parent_frame   = parent_frame
        self.child_frame    = child_frame
        self.recov_path     = recov_path
        self.robot_pose_tf          = Robot_pose_tf(self.parent_frame,self.child_frame) #Transform between Parent Frame and Child Frame
        self.robot_pose_tf_val      = self.robot_pose_tf.pose_tf()
        self.pose_recovery          = Pose_recovery(path=self.recov_path)
        self.pose_recovery_state    = self.pose_recovery.recovery_pose_state()
        self.pose_realtime          = Pose_Realtime(path=RTlog_path)

        self.msg = State_RobotState()
        self.msg.recovery_pose_state = self.pose_recovery_state

    def pose_param_clbk(self):
        return [self.robot_pose_tf_val, self.pose_recovery_state]

    def pose_save(self,state_stop_pub):
        self.robot_pose = self.robot_pose_tf.pose_tf() #Get pose of Robot
        self.pose_realtime.pose_save(self.robot_pose) #Save Robot Pose in realtime
        if state_stop_pub == False:
            self.pub.publish(self.msg)
        else:
            pass

class Node:
    def __init__(self,node_name,rate):
        self.node_name  = node_name
        self.rate       = rate
        rospy.init_node(self.node_name, anonymous=True)
        rospy.Subscriber('pose_state_stop_pub', State_RobotState, self.state_stop_pub_clbk)
        self.pub = rospy.Publisher('pose_robot_tf',Pose_RobotPose,queue_size=10)

    def state_stop_pub_clbk(self,data):
        self.state_stop_pub = data.state_stop_pub
        state_pub_judge(state_stop_pub)

    def state_stop_pub_judge(state_stop_pub):
        if state_stop_pub == 1:
            return True
        else:
            return False

    def pub_robot_pose(self,robot_pose):
        msg = Pose_RobotPose()
        msg.x_euler     = robot_pose[0]
        msg.y_euler     = robot_pose[1]
        msg.yaw_euler   = robot_pose[2]
        self.pub.publish(msg)


if __name__ == '__main__':
    Node        = Node(node_name,rate)

    rate        = rospy.Rate(Node.rate)

    RT_Pose     = RT_Pose(parent_frame,child_frame,recov_path)
    pose_param  = RT_Pose.pose_param_clbk() #Output for [[robot_pose], state_recov]
    while not rospy.is_shutdown():
        Node.pub_robot_pose(pose_param[0]) #Publish pose tf Realtime in topic pose_robot_tf
        RT_Pose.pose_save(Node.state_stop_pub_judge())
        rate.sleep()
