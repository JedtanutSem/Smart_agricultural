#!/usr/bin/env python
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Robot_pose_tf:

    def __init__(self, parent_frame, child_frame):
        self.parent_frame = parent_frame
        self.child_frame  = child_frame
        self.listener = tf.TransformListener()

    def pose_tf(self):
        try:
            (trans,rot) = self.listener.lookupTransform(self.parent_frame,self.child_frame,rospy.Time(0))
            (roll, pitch, yaw) = euler_from_quaternion(rot)

            x =  trans[0]
            y =  trans[1]
            #rospy.loginfo('x: '+ str(x)+ "  y: "+str(y)+ " ang: " + str(angular))
            return [x,y,yaw]
            #rospy.loginfo(msg)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return [0,0,0]
            pass
