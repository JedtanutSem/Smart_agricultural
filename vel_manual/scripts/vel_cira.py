#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
json_data = 0
rate = 0
vel_x = 0
ang_z = 0

def cira_data_clbk(data):
    global json_data,vel_x, ang_z
    get = data.data
    json_acceptable_string = get.replace("'", "\"")
    json_data = json.loads(json_acceptable_string)
    vel_x = json_data['vel_x']
    ang_z = json_data['ang_z']


    #rospy.loginfo(json_data)


def cira_listen():
    global rate, pub
    rospy.init_node('cira_data', anonymous=True)
    rospy.Subscriber('/from_cira', String, cira_data_clbk)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = vel_x
        msg.angular.z = ang_z
        pub.publish(msg)
        rospy.loginfo(msg)
        rate.sleep()

    #rospy.spin()

def show_data():
    rospy.loginfo(json_data)


if __name__ == '__main__':
    cira_listen()
        #show_data()
