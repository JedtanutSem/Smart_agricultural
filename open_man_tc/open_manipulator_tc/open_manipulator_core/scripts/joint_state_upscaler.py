#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointStatesUpscaler(object):

    def __init__(self):
        self.j_s_topic = "/safe/joint_states"
        self.check_joint_states_ready()        
        self.pub = rospy.Publisher(self.j_s_topic, JointState, queue_size=1)
        rospy.Subscriber(self.j_s_topic, JointState, self.callback)


    def check_joint_states_ready(self):
        self.joint_state_msg = None
        while self.joint_state_msg is None and not rospy.is_shutdown():
            try:
                self.joint_state_msg = rospy.wait_for_message(self.j_s_topic, JointState, timeout=1.0)
                rospy.loginfo("Current "+self.j_s_topic+" READY=>" + str(self.joint_state_msg))

            except:
                rospy.logerr("Current "+self.j_s_topic+" not ready yet, retrying for getting joint_states")

        rospy.loginfo("JOINT STATES READY")

    def callback(self,msg):
        self.joint_state_msg = msg
    
    def update_time_joint_states(self):
        h = Header()
        h.stamp = rospy.Time.now()
        self.joint_state_msg.header = h


    def loop(self):

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.update_time_joint_states()
            self.pub.publish(self.joint_state_msg)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('joint_state_uoscaler_node', anonymous=True)
    jt_o = JointStatesUpscaler()
    jt_o.loop()
