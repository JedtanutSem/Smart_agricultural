#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import json
json_data = 0
rate = 0

#test
def cira_data_clbk(data):
    global json_data
    get = data.data
    json_acceptable_string = get.replace("'", "\"")
    json_data = json.loads(json_acceptable_string)

    #rospy.loginfo(json_data)


def cira_listen():
    global rate
    rospy.init_node('cira_data', anonymous=True)
    rospy.Subscriber('/from_cira', String, cira_data_clbk)
    rate = rospy.Rate(10)
    #rospy.spin()

def show_data():
    rospy.loginfo(json_data)


if __name__ == '__main__':
    while not rospy.is_shutdown():
        cira_listen()
        show_data()
        rate.sleep()
