##!/usr/bin/env python
import pandas as pd

class Pose_Realtime:
    def __init__(self, path='/home/agri/agri_ws/src/navigation_system/robot_system/log/RTlog.csv'):
        self.path = path
        pass

    def fail_print_without_exit(self,fail_state):
        print("******************\n\n")
        print(fail_state)
        print("\n\n******************\n\n")
    def pose_save(self,robot_pose_list):
        pose_list = robot_pose_list
        if len(pose_list) == 3:
            pose_dict = {"x":[pose_list[0]],"y":[pose_list[1]],"w":[pose_list[2]]}
            #pose_dataframe = pd.DataFrame(pose_dict, columns= ['x','y','w'])
            pose_dataframe = pd.DataFrame(pose_dict)
            pose_dataframe.to_csv(self.path, header=True,index=True)
            print(pose_dataframe)
        else:
            self.fail_print_without_exit('Please check position list in Realtime Save')
    def pose_read(self):
        pose_dataframe = pd.read_csv(self.path)
        return pose_dataframe

    #def pose_transform(self,parent_)
