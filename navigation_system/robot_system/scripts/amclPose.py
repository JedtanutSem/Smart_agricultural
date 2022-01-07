#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from robot_system.msg import Robotpose
import math
robot_position = [0,0,0]

def amcl_pose_clbk(PoseWithCovarianceStamped):
    global robot_position
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
    robot_position = [x,y,angular]
    """
    msg = Robotpose()
    msg.x_pose = robot_position[0]
    msg.y_pose = robot_position[1]
    msg.w_pose = robot_position[2]
    pub.publish(msg)
    """


def amcl_pose():
    global pub
    rospy.init_node('amclPose_send',anonymous = True)
    rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped, amcl_pose_clbk)
    #pub = rospy.Publisher('/amcl_euler',Robotpose,queue_size = 10)
    rospy.spin()
    #rate = rospy.Rate(10)

    #rate.sleep()



    #rospy.spin()

if __name__ == "__main__":
    amcl_pose()
