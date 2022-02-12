#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

wheeltrack = 0.32
wheelradius = 0.06

def cmd_vel_clbk(data):
    #global
    vel_x = data.linear.x
    ang_z = data.angular.z
    #velocity_right_wheel = ((2*vel_x) + (ang_z*wheeltrack))/(2*wheelradius)
    #velocity_left_wheel = ((2*vel_x) - (ang_z*wheeltrack))/(2*wheelradius)
    velocity_left_wheel = vel_x + (ang_z/2)
    velocity_right_wheel = vel_x - (ang_z/2)

    print('vel_L: '+str(velocity_left_wheel) + 'vel_R: '+str(velocity_right_wheel) )



def robot_vel_transform():
    rospy.init_node('robot_vel_transform', anonymous = True)
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_clbk)
    rospy.spin()

if __name__ == "__main__":
    robot_vel_transform()
