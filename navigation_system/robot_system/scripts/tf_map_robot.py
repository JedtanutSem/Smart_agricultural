#!/usr/bin/env python

import rospy
import math
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler


from robot_system.msg import Robotpose

if __name__ == "__main__":
    rospy.init_node("tf_robot")
    listener = tf.TransformListener()
    pose_pub = rospy.Publisher('robot/position',Robotpose,queue_size = 10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map','/base_footprint',rospy.Time(0))
            (roll, pitch, yaw) = euler_from_quaternion (rot)
            angular = yaw * (180/math.pi)

            x =  trans[0]
            y = trans[1]
            rospy.loginfo('x: '+ str(x)+ "  y: "+str(y)+ " ang: " + str(angular))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()
