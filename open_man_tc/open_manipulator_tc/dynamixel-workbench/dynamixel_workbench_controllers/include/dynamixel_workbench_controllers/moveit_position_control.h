/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#ifndef DYNAMIXEL_WORKBENCH_MOVEIT_POSITION_CONTROL_H
#define DYNAMIXEL_WORKBENCH_MOVEIT_POSITION_CONTROL_H

#include <ros/ros.h>

#include "message_header.h"

#include <sensor_msgs/JointState.h>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/JointCommand.h>
#include <my_moveit_planner/JointStatesValidationServiceMessage.h>

#include <iostream>
#include <exception>

class MoveitPositionControl
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher dynamixel_state_list_pub_;
  ros::Publisher joint_states_pub_;

  // ROS Topic Subscriber
  ros::Subscriber joint_command_sub_;
  ros::Subscriber joint_states_sub_;

  // ROS Service Server
  ros::ServiceServer joint_command_server_;

  // ROS Service Client
  ros::ServiceClient joint_states_validation_client_;
  

  // Dynamixel Workbench Parameters
  DynamixelWorkbench *dxl_wb_;
  uint8_t dxl_id_[16];
  uint8_t dxl_cnt_;

  bool join_states_published_once;

  sensor_msgs::JointState current_joint_states;

 public:
  MoveitPositionControl();
  ~MoveitPositionControl();
  void controlLoop(void);

  //DECLARE the pointer
  bool success;

 private:
  void initMsg();


  void initPublisher();
  void initSubscriber();
  void dynamixelStatePublish();
  void jointStatePublish();

  void initServer();
  void initClient();
  bool jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                               dynamixel_workbench_msgs::JointCommand::Response &res);
  void goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void jointstateCallback(const sensor_msgs::JointState::ConstPtr &msg);
};

#endif //DYNAMIXEL_WORKBENCH_MOVEIT_POSITION_CONTROL_H
