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

/* Authors: Taehun Lim (Darby), MiguelAngelRodriguez ( mod of Service ) */

#include "dynamixel_workbench_controllers/moveit_position_control.h"

MoveitPositionControl::MoveitPositionControl()
    :node_handle_("")
{

  std::string device_name   = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  uint32_t dxl_baud_rate    = node_handle_.param<int>("baud_rate", 57600);

  uint8_t scan_range        = node_handle_.param<int>("scan_range", 200);

  uint32_t profile_velocity     = node_handle_.param<int>("profile_velocity", 200);
  uint32_t profile_acceleration = node_handle_.param<int>("profile_acceleration", 50);

  dxl_wb_ = new DynamixelWorkbench;

  dxl_wb_->begin(device_name.c_str(), dxl_baud_rate);
  
  if (dxl_wb_->scan(dxl_id_, &dxl_cnt_, scan_range) != true)
  {
    ROS_ERROR("Not found Motors, Please check scan range or baud rate");
    ros::shutdown();
    return;
  }

  initMsg();

  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_->jointMode(dxl_id_[index], profile_velocity, profile_acceleration);

  dxl_wb_->addSyncWrite("Goal_Position");
  printf("  initPublisher             \n");
  initPublisher();
  printf("  initSubscriber             \n");
  initSubscriber();
  printf("  initServer             \n");
  initServer();
  printf("  initClient            \n");
  initClient();
  printf("  END INIT             \n");
}

MoveitPositionControl::~MoveitPositionControl()
{
  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);

  ros::shutdown();
}

void MoveitPositionControl::initMsg()
{
  printf("-----------------------------------------------------------------------\n");
  printf("  dynamixel_workbench controller; position control example             \n");
  printf("-----------------------------------------------------------------------\n");
  printf("\n");

  for (int index = 0; index < dxl_cnt_; index++)
  {
    printf("MODEL   : %s\n", dxl_wb_->getModelName(dxl_id_[index]));
    printf("ID      : %d\n", dxl_id_[index]);
    printf("\n");
  }
  printf("-----------------------------------------------------------------------\n");
}

void MoveitPositionControl::initPublisher()
{
  dynamixel_state_list_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 10);
  joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 10);
}

void MoveitPositionControl::initSubscriber()
{
  joint_command_sub_ = node_handle_.subscribe("goal_dynamixel_position", 10, &MoveitPositionControl::goalJointPositionCallback, this);
  joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &MoveitPositionControl::jointstateCallback, this);
}

void MoveitPositionControl::initServer()
{
  joint_command_server_ = node_handle_.advertiseService("joint_command", &MoveitPositionControl::jointCommandMsgCallback, this);
}

void MoveitPositionControl::initClient()
{
  joint_states_validation_client_ = node_handle_.serviceClient<my_moveit_planner::JointStatesValidationServiceMessage>("joint_state_validator");
}

void MoveitPositionControl::dynamixelStatePublish()
{
  dynamixel_workbench_msgs::DynamixelState     dynamixel_state[dxl_cnt_];
  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list;

  for (int index = 0; index < dxl_cnt_; index++)
  {
    dynamixel_state[index].model_name          = std::string(dxl_wb_->getModelName(dxl_id_[index]));
    dynamixel_state[index].id                  = dxl_id_[index];
    dynamixel_state[index].torque_enable       = dxl_wb_->itemRead(dxl_id_[index], "Torque_Enable");
    dynamixel_state[index].present_position    = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");
    dynamixel_state[index].present_velocity    = dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");
    dynamixel_state[index].goal_position       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Position");
    dynamixel_state[index].goal_velocity       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Velocity");
    dynamixel_state[index].moving              = dxl_wb_->itemRead(dxl_id_[index], "Moving");

    dynamixel_state_list.dynamixel_state.push_back(dynamixel_state[index]);
  }
  dynamixel_state_list_pub_.publish(dynamixel_state_list);
}

void MoveitPositionControl::jointStatePublish()
{
  int32_t present_position[dxl_cnt_] = {0, };

  for (int index = 0; index < dxl_cnt_; index++)
    present_position[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");

  int32_t present_velocity[dxl_cnt_] = {0, };

  for (int index = 0; index < dxl_cnt_; index++)
    present_velocity[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");

  sensor_msgs::JointState dynamixel_;
  dynamixel_.header.stamp = ros::Time::now();

  for (int index = 0; index < dxl_cnt_; index++)
  {
    std::stringstream id_num;
    id_num << "id_" << (int)(dxl_id_[index]);

    dynamixel_.name.push_back(id_num.str());

    dynamixel_.position.push_back(dxl_wb_->convertValue2Radian(dxl_id_[index], present_position[index]));
    dynamixel_.velocity.push_back(dxl_wb_->convertValue2Velocity(dxl_id_[index], present_velocity[index]));
  }

  // We publish the gripper position joints to be compliant with the moveit package
  for (int index = 8; index < 10; index++)
  {
    std::stringstream id_num;
    id_num << "joint" << (int)(index);

    dynamixel_.name.push_back(id_num.str());

    // False values not important
    dynamixel_.position.push_back(-0.0110133334);
    dynamixel_.velocity.push_back(0.0);
  }

  joint_states_pub_.publish(dynamixel_);
}

void MoveitPositionControl::controlLoop()
{
  dynamixelStatePublish();
  jointStatePublish();
}

bool MoveitPositionControl::jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                                              dynamixel_workbench_msgs::JointCommand::Response &res)
{
  int32_t goal_position = 0;
  int32_t present_position = 0;
  bool ret = false;
  // We first chekc that the ID is OK
  // - id_1
  // - id_2
  // - id_3
  // - id_4
  // - id_5
  // - id_6
  ROS_WARN("req.id=%d", req.id);
  if (0 < req.id && req.id <= 7){
    ROS_WARN("Joint ID is between 1-7..OK");

    // We create the message and add the modifications
    // We Validate with a service
    my_moveit_planner::JointStatesValidationServiceMessage srv;
    srv.request.joint_states_data.header = this->current_joint_states.header;
    srv.request.joint_states_data.name = this->current_joint_states.name;

    srv.request.joint_states_data.position = this->current_joint_states.position;
    // We update the value with the asked one
    int index = req.id - 1;
    ROS_WARN("index=%d", index);
    srv.request.joint_states_data.position[index] = req.goal_position;

    srv.request.joint_states_data.velocity = this->current_joint_states.velocity;
    srv.request.joint_states_data.effort = this->current_joint_states.effort;

    if (joint_states_validation_client_.call(srv)) // Send through the connection the name of the trajectory to execute
    {
      ROS_INFO("Status Validation=%s", srv.response.status_message.c_str());

      if (srv.response.valid){
        ROS_WARN("Joints OK...Sending");

        
        
        if (req.unit == "rad")
        {
          goal_position = dxl_wb_->convertRadian2Value(req.id, req.goal_position);
        }
        else if (req.unit == "raw")
        {
          goal_position = req.goal_position;
        }
        else
        {
          goal_position = req.goal_position;
        }

        ret = dxl_wb_->goalPosition(req.id, goal_position);


      }else{
        ROS_ERROR("Joints DANGEROUS...NOT SENDING");
      }
    }
    else
    {
      ROS_ERROR("Failed to call service joint_state_validator");
    }


    




  }else{
    ROS_ERROR("Joint ID is NOT between 1-6..ERROR");
  }  

  res.result = ret;
}


void MoveitPositionControl::jointstateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{

  this->current_joint_states.header = msg->header;
  this->current_joint_states.name = msg->name;
  this->current_joint_states.position = msg->position;
  this->current_joint_states.velocity = msg->velocity;
  this->current_joint_states.effort = msg->effort;

}

void MoveitPositionControl::goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg)
{

  // We Validate with a service
  my_moveit_planner::JointStatesValidationServiceMessage srv;
  srv.request.joint_states_data.header = msg->header;
  srv.request.joint_states_data.name = msg->name;
  srv.request.joint_states_data.position = msg->position;
  srv.request.joint_states_data.velocity = msg->velocity;
  srv.request.joint_states_data.effort = msg->effort;

  
  if (joint_states_validation_client_.call(srv)) // Send through the connection the name of the trajectory to execute
  {
    ROS_INFO("Status Validation=%s", srv.response.status_message.c_str());

    if (srv.response.valid){
      ROS_WARN("Joints OK...Sending");
      double goal_position[dxl_cnt_] = {0.0, };

      for (int index = 0; index < dxl_cnt_; index++)
        goal_position[index] = msg->position.at(index);

      int32_t goal_dxl_position[dxl_cnt_] = {0, };

      for (int index = 0; index < dxl_cnt_; index++)
      {
        goal_dxl_position[index] = dxl_wb_->convertRadian2Value(dxl_id_[index], goal_position[index]);
      }

      dxl_wb_->syncWrite("Goal_Position", goal_dxl_position);


    }else{
      ROS_ERROR("Joints DANGEROUS...NOT SENDING");
    }

    

    

    
  }
  else
  {
    ROS_ERROR("Failed to call service joint_state_validator");
  }



  

}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "moveit_position_control");

  MoveitPositionControl pos_ctrl;  
  ros::Rate loop_rate(250);

  printf("  Start FOR loop main             \n");
  while (ros::ok())
  {
    pos_ctrl.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
