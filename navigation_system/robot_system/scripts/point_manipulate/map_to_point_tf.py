#!/usr/bin/env python
import rospy
import tf
import json
from std_msgs.msg import String
from robot_system.msg import Robotpose
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

position = [0,0,0]

def cira_pose_clbk(msg):
    global position
    goal_str = msg.data
    json_acceptable_string = goal_str.replace("'", "\"")
    goal_val = json.loads(json_acceptable_string)
    x_pose = goal_val['x_pose']
    y_pose = goal_val['y_pose']
    yaw_pose = goal_val['yaw_pose']
    yaw_pose = yaw_pose * (math.pi /180)

    position = [x_pose, y_pose, yaw_pose]
    #rospy.loginfo('x : '+ str(x_pose)+"  y: "+str(y_pose) + ' yaw: '+str(yaw_pose))




def cira_pose_listen():
    rospy.init_node('cira_pose_listen')
    rospy.Subscriber('/pose_cira',String, cira_pose_clbk)
    pub = rospy.Publisher('/tf_map_home',Robotpose,queue_size = 10)
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        quatern_list = tf.transformations.quaternion_from_euler(0,0,position[2])
        br.sendTransform((position[0],position[1],0),(quatern_list[0],quatern_list[1],quatern_list[2],quatern_list[3]),rospy.Time.now(),"point", "home")
        #rospy.loginfo("teest")
        try:
            #listener = tf.TransformListener()
            (trans,rot) = listener.lookupTransform('/map','/point',rospy.Time(0))
            rospy.loginfo(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        msg = Robotpose()
        msg.x_trans = trans[0]
        msg.y_trans = trans[1]
        msg.z_trans = trans[2]

        msg.x_rot = rot[0]
        msg.y_rot = rot[1]
        msg.z_rot = rot[2]
        msg.w_rot = rot[3]
        pub.publish(msg)
        #rospy.loginfo(msg)
        rate.sleep()

if __name__ == "__main__":
    cira_pose_listen()
