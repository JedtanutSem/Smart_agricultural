#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json


Raduis_wheel = 0.035

kp = 0
ki = 0
kd = 0
past_time = 0
current_time = 0

targetvelocity_left = -0.2
targetvelocity_right = 0.00002

current_velocity_left = 0
current_velocity_right = 0

delta_time_left = 1.2
delta_time_right = 1.2

direction_left = 0
direction_right = 0

wheelbase = 0.32


targetvelocity_left = 0
targetvelocity_right = 0

current_velocity_left = 0
current_velocity_right = 0

eintegral_left = 0.0
eprev_left = 0.0

eintegral_right = 0.0
eprev_right = 0.0


def pid_val_clbk(data):
    global kp,ki,kd
    PID_str = data.data
    json_acceptable_string = PID_str.replace("'", "\"")
    PID_dict = json.loads(json_acceptable_string)
    #print PID_dict

    kp = float(PID_dict['kp'])
    ki = float(PID_dict['ki'])
    kd = float(PID_dict['kd'])

def cmd_vel_clbk(data):
    global targetvelocity_left, targetvelocity_right
    vel_x = data.linear.x
    ang_z = data.angular.z
    targetvelocity_left = vel_x - ang_z*(wheelbase/2)
    targetvelocity_right = vel_x + ang_z*(wheelbase/2)


def wheel_vel_clbk(data):
    global current_velocity_left, current_velocity_right
    msg_str = data.data
    msg_list = msg_str.split(',')
    current_velocity_left = float(msg_list[0])
    current_velocity_right = float(msg_list[1])
    #print(current_velocity_left)

def pid(current_velocity,target_velocity,deltaT,position):
    """
    global kp
    global ki
    global kd
    """

    direction = 0

    if position == "left" :
        global eintegral_left
        global eprev_left


        #e = current_velocity - target_velocity
        e = target_velocity - current_velocity
        #e = 0

        dedt = (e-eprev_left)/(deltaT)

        eintegral_left = eintegral_left + e*deltaT

        u = kp*e + kd*dedt + ki*eintegral_left
        #u = kp*e
        eprev_left = e
        """
        if(target_velocity == 0):
            u = 0
            eintegral_left = 0
            """
        #print("left: "+str((current_velocity - target_velocity)))
        #print(str(e)+' = ' + str(current_velocity)+' - '+ str(target_velocity))
        print(str(u)+' = ' + str(kp*e)+' + '+ '('+str(ki)+'*'+str(eintegral_left))
        #print ('Left:  '+str(eintegral_left))

    if position == "right" :
        global eintegral_right
        global eprev_right

        e = current_velocity - target_velocity

        dedt = (e-eprev_right)/(deltaT)

        eintegral_right = eintegral_right + e*deltaT

        u = kp*e + kd*dedt + ki*eintegral_right

        eintegral_right = e
        u=0
        #print("right: "+str((current_velocity - target_velocity)))
        #u = 0

        #print ('Right: '+str(eintegral_right))

    #print('Left: '+ str(eintegral_left) + 'Right: '+str(en) )
    #print e
    if u > 0 : direction = 1
    #if u < 10 and u > -10 :u = 0
    #print(u)
    pwm = abs(0)
    direction = 1
    if pwm > 255 : pwm = 255
    if pwm < 10  : pwm = 0

    return pwm,direction

#loop_time = time.time()


if __name__ == "__main__":
    time.sleep(3)
    rospy.init_node('robot_vel_PID', anonymous=True)
    rospy.Subscriber('wheel_vel', String, wheel_vel_clbk)
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_clbk)
    rospy.Subscriber('/PID_val', String, pid_val_clbk)
    pub_pwm = rospy.Publisher('pwm_to_controller', String, queue_size = 10)
    current_time = rospy.Time.now().to_sec()
    past_time = rospy.Time.now().to_sec()
    loop_time = rospy.Time.now().to_sec()
    #print(current_time)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #recive_data

        #current_time = time.time()
        current_time = rospy.Time.now().to_sec()

        delta_time = current_time - past_time
        past_time = current_time

        #if True :

        try:
            pwm_left,direction_left = pid(current_velocity_left,targetvelocity_left,delta_time,"left")
            pwm_right,direction_right = pid(current_velocity_right,targetvelocity_right,delta_time,"right")
            pwm = str(pwm_left)+","+str(direction_left)+","+str(pwm_right)+","+str(direction_right)
            #print(pwm)
            pwm_msg = String()
            pwm_msg.data = pwm
            pub_pwm.publish(pwm_msg)
            #print(pwm_msg) # 0 => reverse ;  1 => forward
            #print(delta_time)
            loop_time = rospy.Time.now().to_sec()
        except Exception as e:
            print(e)
        rate.sleep()
