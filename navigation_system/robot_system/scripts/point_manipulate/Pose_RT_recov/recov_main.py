#!/usr/bin/env python

import rospy
import os
import time
from realtime_pose import Pose_Realtime
from pose_recovery import Pose_recovery
from robot_msg.msg import State_RobotState
from robot_pose_tf import Robot_pose_tf

node_name           = 'Pose_RTsave'
rate                = rospy.get_param('rate_RT_Save', 2)

parent_frame        = rospy.get_param('parent_frame', '/map')
child_frame         = rospy.get_param('child_frame' , '/base_footprint')

file_path           = '/home/agri/agri_ws/src/navigation_system/robot_system/log/recov_state.csv'

recov_path          = rospy.get_param('recov_path', file_path)

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
        self.pose_realtime          = Pose_Realtime()

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

    def state_stop_pub_clbk(self,data):
        self.state_stop_pub = data.state_stop_pub
        state_pub_judge(state_stop_pub)

    def state_pub_judge(state_stop_pub):
        if state_stop_pub == 1:
            return True
        else:
            return False

if __name__ == '__main__':
    Node        = Node(node_name,rate)
    state_pub   = Node.state_pub_judge()

    rate        = rospy.Rate(Node.rate)

    RT_Pose     = RT_Pose(parent_frame,child_frame,recov_path)
    pose_param  = RT_Pose.pose_param_clbk()
    #print(pose_param)
    while not rospy.is_shutdown():
        RT_Pose.pose_save(state_pub)
        rate.sleep()
