#!/usr/bin/env python

import math
from math import sin, cos, pi
import serial

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16,String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String
import json


#Parameters
wheeltrack = 0.32 #distace measure of distance between wheel
wheelradius = 0.06
TPR = 520 #TPR --> Tick per revolution
index_vl = 0
index_vr = 0
index_vx = 0
index_vy = 0
index_vth = 0

left_ticks = 32768
right_ticks = 32768
last_left_ticks = 32768
last_right_ticks = 32768

x = 0.0
y = 0.0
th = 0.0

vx =  0.0
vy =  0.0
vth =  0.0

vx_send = 0
vy_send = 0
vth_send = 0

vl_send = 0
vr_send = 0


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
wheel_vel_pub = rospy.Publisher("wheel_vel", String, queue_size = 50)
odom_broadcaster = tf.TransformBroadcaster()
current_time = rospy.Time.now()
last_time = rospy.Time.now()


r = rospy.Rate(50)

while not rospy.is_shutdown():



    current_time = rospy.Time.now()
    #print(left_ticks)
    delta_L = left_ticks - last_left_ticks
    delta_R = right_ticks - last_right_ticks
    if(delta_L > 0):
        if(delta_L > 1000):
            delta_L = last_delta_L
    if(delta_L < 0):
        if(delta_L < -1000):
            delta_L = last_delta_L
    if(delta_R > 0):
        if(delta_R > 1000):
            delta_R = last_delta_R
    if(delta_R < 0):
        if(delta_R < -1000):
            delta_R = last_delta_R

    last_delta_L = delta_L
    last_delta_R = delta_R
    #if(delta_L != 0 or delta_R!=0):
        #print('delta;L '+str(delta_L)+'    delta;R '+str(delta_R))
    last_left_ticks = left_ticks
    last_right_ticks = right_ticks
    #global last_left_ticks
    #global last_right_ticks
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
        #dx = dc*cos(th+(dth/2))
        #dx = dc*sin(th+(dth/2))

        dx = cos(dth) * (x-iccX) - sin(dth) * (y-iccY) + iccX - x
        dy = sin(dth) * (x-iccX) + cos(dt) * (y-iccY) + iccY - y

    x += dx
    y += dy
    #print(dx)
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
    if(vx!=0):
        vx_send = vx
        index_vx = 0
    elif(vx==0):
        index_vx = index_vx+1
        if(index_vx == 10):
            vx_send = 0

    if(vy!=0):
        vy_send = vy
        index_vy = 0
    elif(vy==0):
        index_vy = index_vy+1
        if(index_vy == 10):
            vy_send = 0

    if(vth!=0):
        vth_send = vth
        index_vth = 0
    elif(vth==0):
        index_vth = index_vth+1
        if(index_vth == 10):
            vth_send = 0
    odom.twist.twist = Twist(Vector3(vx_send, vy_send, 0), Vector3(0, 0, vth_send))
    vl = dl/dt
    vr = dr/dt
    #if(vl!=0 or vr!=0):
        #print('v_L: '+str(vl)+'    v_RLL '+str(vr))
    #if(vl == 0):
        #print('v_L: '+str(vl)+'    v_RLL '+str(vr))
    #print('v_L: '+str(vl)+'    v_R '+str(vr))
    if(vl != 0):
        vl_send = vl
        #print('v_L: '+str(vl)+'    v_R '+str(vr))
        index_vl = 0
        #pass
    elif(vl == 0):
        index_vl = index_vl+1
        if(index_vl == 10):
            vl_send = vl
            #print('v_L: '+str(0)+'    v_R '+str(0))
            index_vl = 0


    if(vr != 0):
        vr_send = vr
        #print('v_L: '+str(vl)+'    v_R '+str(vr))
        index_vr = 0
        #pass
    elif(vr == 0):
        index_vr = index_vr+1
        if(index_vr == 10):
            vr_send = vr
            #print('v_L: '+str(0)+'    v_R '+str(0))
            index_vr = 0
    wheel_vel_msg = String()
    wheel_vel_msg.data = "%s,%s" %(vl_send,vr_send)
    #print(rospy.Time.now().to_sec())
    print(wheel_vel_msg)
    wheel_vel_pub.publish(wheel_vel_msg)



    odom_pub.publish(odom)


    #rospy.loginfo("left %d and right %d",left_ticks,right_ticks)
    last_time = current_time
    r.sleep()
