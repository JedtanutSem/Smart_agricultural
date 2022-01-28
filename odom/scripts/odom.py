#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16,String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import json


#Parameters
wheeltrack = 0.143 #distace measure of distance between wheel
wheelradius = 0.0325
TPR = 27000 #TPR --> Tick per revolution
left_ticks = 0
right_ticks = 0
last_left_ticks = 0
last_right_ticks = 0

x = 0.0
y = 0.0
th = 0.0

vx =  0.0
vy =  0.0
vth =  0.0

def encoder_callback(msg):
    global left_ticks
    global right_ticks
    ticks = msg.data
    ticks = ticks.split(",")
    left_ticks_str = ticks[0]
    left_ticks = float(left_ticks_str)
    right_ticks_str = ticks[1]
    right_ticks = float(right_ticks_str)


rospy.init_node('odometry_publisher')
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50) # odom published
encoder_sub =rospy.Subscriber("/serial_read", String, encoder_callback) # Encoder count from serial(Controller)
odom_broadcaster = tf.TransformBroadcaster()

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(150)

while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    delta_L = left_ticks - last_left_ticks
    delta_R = right_ticks - last_right_ticks
    last_left_ticks = left_ticks
    last_right_ticks = right_ticks
    dl = 2 * pi * wheelradius * delta_L / TPR # distance of left motor
    dr = 2 * pi * wheelradius * delta_R / TPR # distance of righr motor
    dc = (dl + dr) / 2 # center distance
    dt = (current_time - last_time).to_sec()
    dth = (dr-dl)/wheeltrack # angular distance (theta)

    if dr==dl: # when the robot doesn't has angular velocity
        dx=dr*cos(th)
        dy=dr*sin(th)

    else:
        radius=dc/dth   # curve(dc) = r*theta(dth)
        #<--calculate icc value-->
        iccX=x-radius*sin(th)
        iccY=y+radius*cos(th)

        dx = cos(dth) * (x-iccX) - sin(dth) * (y-iccY) + iccX - x
        dy = sin(dth) * (x-iccX) + cos(dt) * (y-iccY) + iccY - y

    x += dx
    y += dy
    th =(th+dth) %  (2 * pi) # euler angle round z axis

    # <--Quaternion coordinate odom-->
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th) # only z axis (robot rotate)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
       (x, y, 0.),
       odom_quat,
       current_time,
       "base_link",
       "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    if dt>0:
       vx=dx/dt
       vy=dy/dt
       vth=dth/dt

    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    odom_pub.publish(odom)


    #rospy.loginfo("left %d and right %d",left_ticks,right_ticks)
    last_time = current_time
    r.sleep()
