##!/usr/bin/env python
import rospy
import pandas as pd
from robot_msg.msg import Robot_state

class Pose_recovery:
    def __init__(self,path='/home/agri/agri_ws/src/navigation_system/robot_system/log/recov_state.csv'):
        #rospy.Subscriber('recovery_pose_state', Robot_state, self.recovery_pose_state_clbk)
        self.pub_recovery = rospy.Publisher('recovery_pose_state',Robot_state,queue_size=10)
        self.path = path
        self.read_trigger = 0
        #self.recovery_state = 0
    def recovery_pose_state(self):
        if self.read_trigger == 0:
            recov_state_dataframe = pd.read_csv(self.path)
            recov_state = recov_state_dataframe.iloc[0][1]
            #print('VAl '+str(recov_state_dataframe.iloc[0][1]))
            if recov_state == 0:
                #print('not recovery')
                self.read_trigger = 1
                return False
            else:
                #print('recovery')
                self.read_trigger = 0
                return True
        else:
            pass
            return False
        """
        try:
            recovery_pose_state = self.recovery_state
            if recovery_pose_state == 1:
                print('True')
                return True

            else:
                print('False')

            #return recovery_pose_state

        except AttributeError:
            pass
        except Exception as e:
            #return e
            print('Error: '+str(e))
        """
