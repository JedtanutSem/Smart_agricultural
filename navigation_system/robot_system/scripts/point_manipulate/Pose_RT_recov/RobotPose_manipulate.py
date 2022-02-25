#!/usr/bin/env python

import rospy
import os
from realtime_pose import Pose_Realtime
from pose_recovery import Pose_recovery
from robot_msg.msg import Robot_state
from robot_pose_tf import Robot_pose_tf

parent_frame        = '/map'
child_frame         = '/base_footprint'

#path =

if __name__ == '__main__':
    rospy.init_node('RobotPose_manipulate', anonymous=True)
    robot_pose_tf   = Robot_pose_tf(parent_frame,child_frame)
    pose_recovery   = Pose_recovery()
    pose_realtime   = Pose_Realtime()
    #a = pose_save.pose_read()
    recovery_state = pose_recovery.recovery_pose()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        robot_pose = robot_pose_tf.pose_tf()
        pose_realtime.pose_save(robot_pose)
        #print(robot_pose_tf.pose_tf())
        #recovery_state = pose_recovery.recovery_pose()
        #print('KDKD  '+str(recovery_state))
        rate.sleep()
