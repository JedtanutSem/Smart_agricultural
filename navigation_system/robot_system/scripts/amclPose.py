#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

def amcl_pose_clbk(PoseWithCovarianceStamped):
    x = PoseWithCovarianceStamped.pose.pose.position.x
    y = PoseWithCovarianceStamped.pose.pose.position.y
    x_rot = PoseWithCovarianceStamped.pose.pose.orientation.x
    y_rot = PoseWithCovarianceStamped.pose.pose.orientation.y
    z_rot = PoseWithCovarianceStamped.pose.pose.orientation.z
    w_rot = PoseWithCovarianceStamped.pose.pose.orientation.w
    rot = [x_rot,y_rot,z_rot,w_rot]
    (roll, pitch, yaw) = euler_from_quaternion(rot)
    angular = yaw * (180/math.pi)
    #angular = rot
    rospy.loginfo('x: '+ str(x)+ "  y: "+str(y)+ " ang: " + str(angular))


def amcl_pose_listen():
    rospy.init_node('amclPose_send',anonymous = True)
    rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped, amcl_pose_clbk)
    rospy.spin()

if __name__ == "__main__":
    amcl_pose_listen()
