#!/usr/bin/env python
import rospy
import json
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String
x_1 = 0
y_1 = 0
def cira_core_tf_clbk(data):
    global x_1, y_1
    data_get_str = data.data
    json_acceptable_string = data_get_str.replace("'", "\"")
    data_get_json = json.loads(json_acceptable_string)
    point1 = data_get_json["point_1"]
    point2 = data_get_json["point_2"]
    point3 = data_get_json["point_3"]
    x_1 = point1[0]
    y_1 = point1[1]
    x_2 = point2[0]
    y_2 = point2[1]
    x_3 = point3[0]
    y_3 = point3[1]

    x_1 = x_1*float(0.103769)
    x_1 = x_1*0.01
    #print(dis_pixel*0.103769
    y_1 = y_1*float(0.103769)
    y_1 = y_1*0.01


    rospy.loginfo(x_1)

def cira_core_tf():
    rospy.init_node("cira_core_tf", anonymous = True)
    rospy.Subscriber("cira_tf_xy",String, cira_core_tf_clbk)
    br = tf.TransformBroadcaster()
    #rospy.spin()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        quatern_list = tf.transformations.quaternion_from_euler(0,0,0)
        br.sendTransform((x_1,1,0),(quatern_list[0],quatern_list[1],quatern_list[2],quatern_list[3]),rospy.Time.now(),"object", "car")
        #rospy.loginfo("teest")

if __name__ == "__main__":
    cira_core_tf()
