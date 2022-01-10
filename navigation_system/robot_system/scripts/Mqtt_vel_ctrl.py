#!/usr/bin/env python


import rospy
import json
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def mqtt_vel_clbk(data):
    data_str = data.data
    json_acceptable_string = data_str.replace("'", "\"")
    data_dict = json.loads(json_acceptable_string)
    x_vel = data_dict["x_vel_state"]
    w_vel = data_dict["w_vel_state"]
    manual_vel_trig = data_dict["manual_vel_trig"]
    if(manual_vel_trig == 1):
        try:
            msg = Twist()
            msg.linear.x = x_vel
            msg.angular.z = w_vel
            pub.publish(msg)
        except:
            rospy.loginfo("Please Restart")

def cira_vel_listen():
    global pub
    rospy.init_node("Mqtt_vel_ctrl",anonymous = True)
    rospy.loginfo("Starting Node-RED Velocity control")
    rospy.Subscriber("Mqtt_vel_send", String, mqtt_vel_clbk)
    pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
    rospy.spin()

if __name__ == "__main__":
    cira_vel_listen()
