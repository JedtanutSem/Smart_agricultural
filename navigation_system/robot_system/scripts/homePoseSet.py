#!/usr/bin/env python
import pandas as pd
import rospy
import json
import os
from std_msgs.msg import String
from robot_system.msg import Robotpose

def poseCLBK(data):
    homePose_str = data.data
    json_acceptable_string = homePose_str.replace("'", "\"")
    homePose_dict_get = json.loads(json_acceptable_string)
    length = len(homePose_dict_get['homePose']['x'])
    #rospy.loginfo(length)
    #for i in range(length):
        #rospy.loginfo("x"+str(i)+" : " +str(homePose_dict_get['homePose']['x'][i]))

        #homePose_dict = {'x':[], 'y':[], 'w':w}
    x = homePose_dict_get['homePose']['x']
    y = homePose_dict_get['homePose']['y']
    w = homePose_dict_get['homePose']['w']
    #rospy.loginfo(type(x))
    """
    x = [x]
    y = [y]
    w = [w]
    """
    msg = HomePose()
    msg.x_pose = x[0]
    msg.y_pose = y[0]
    msg.w_pose = w[0]

    #rospy.loginfo(str(x)+"  "+str(y)+"  "+str(w))
    try:
        """
        file = '/home/jedtanut/agri_ws/src/navigation_system/robot_system/log/homePose.csv'
        if(os.path.exists(file) and os.path.isfile(file)):
            os.remove(file)
            print("file deleted")
        else:
            print("file not found")
        """
        homePose_dict = {'x': x, 'y': y, 'w':w}
        homePose = pd.DataFrame(homePose_dict, columns= ['x','y','w'])
        #print(homePose)
        #rospy.loginfo(homePose_dict)
        #print(homePose)
        homePose.to_csv('~/agri_ws/src/navigation_system/robot_system/log/homePose.csv')
        #rospy.loginfo(homePose)
        log_print = "Saved: "+str(x)+"  "+str(y)+"  "+str(w)
        rospy.loginfo(log_print)
        msg.fail_state = "Saved"
        pub.publish(msg)

    except:
        rospy.loginfo("Fail to saved")
        msg.fail_state = "Fail to saved"
        pub.publish(msg)

    #rospy.loginfo("Saved: "+str(x)+"  "+str(y)+"  "+str(w))


def homePoseListen():
    global pub
    rospy.init_node('homePoseToCSV',anonymous = True)
    rospy.loginfo("Starting homePoseToCSV Node")
    rospy.Subscriber('/homePose', String, poseCLBK)
    pub = rospy.Publisher('/homePoseSavedData',Robotpose, queue_size=10)
    rospy.spin()

if __name__ == "__main__":
    homePoseListen()
