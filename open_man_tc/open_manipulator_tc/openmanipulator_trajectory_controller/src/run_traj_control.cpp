// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <dynamixel_workbench_msgs/JointCommand.h>

// Our Action interface type for moving TIAGo's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;


namespace open_manipulator {

  class ArmTrajController {
  public:
    ArmTrajController(ros::NodeHandle& nh,
                 ros::NodeHandle& pnh);

    virtual ~ArmTrajController();

    void createArmClient(arm_control_client_Ptr& actionClient);
    void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal);

    void run();

    arm_control_client_Ptr ArmClient;
    sensor_msgs::JointState current_joint_state;
    

  protected:

    void goalposCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void jointstatesCallback(const sensor_msgs::JointState::ConstPtr& msg);

    bool jointcommandServiceCallback(dynamixel_workbench_msgs::JointCommand::Request& req, 
                                    dynamixel_workbench_msgs::JointCommand::Response& res);
    

    void start();
    void stop();

    ros::NodeHandle& _nh, _pnh;
    ros::CallbackQueue _cbQueue;

    

    double _rate;

    // ROS subscribers
    ros::Subscriber _goalposSub;
    ros::Subscriber _jointstatesSub;

    // Services
    ros::ServiceServer _joint_command_service;
    

  };

  ArmTrajController::ArmTrajController(ros::NodeHandle& nh,
                                     ros::NodeHandle& pnh):
    _nh(nh),
    _pnh(pnh),
    _rate(5.0)
  {
    _nh.setCallbackQueue(&_cbQueue);

    pnh.param<double>("rate", _rate, _rate);

    ROS_INFO_STREAM("The node will operate at maximum " << _rate << " Hz");

    this->createArmClient(this->ArmClient);

  }

  ArmTrajController::~ArmTrajController()
  {
  }


  // Create a ROS action client to move TIAGo's arm
  void ArmTrajController::createArmClient(arm_control_client_Ptr& actionClient)
  {
    ROS_INFO("Creating action client to arm controller ...");

    actionClient.reset( new arm_control_client("arm/follow_joint_trajectory") );

    int iterations = 0, max_iterations = 3;
    // Wait for arm controller action server to come up
    while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
    {
      ROS_DEBUG("Waiting for the arm_controller_action server to come up");
      ++iterations;
    }

    if ( iterations == max_iterations )
      throw std::runtime_error("Error in createArmClient: arm controller action server not available");

    ROS_INFO("Creating action client to arm controller ...READY");

  }


  // Generates a simple trajectory with two waypoints to move TIAGo's arm 
  void ArmTrajController::waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal)
  {
    ROS_INFO("Start Waypoint");

    // The joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("id_1");
    goal.trajectory.joint_names.push_back("id_2");
    goal.trajectory.joint_names.push_back("id_3");
    goal.trajectory.joint_names.push_back("id_4");
    goal.trajectory.joint_names.push_back("id_5");
    goal.trajectory.joint_names.push_back("id_6");

    // Two waypoints in this goal trajectory
    goal.trajectory.points.resize(2);

    // First trajectory point
    // Positions
    int index = 0;
    goal.trajectory.points[index].positions.resize(6);
    goal.trajectory.points[index].positions[0] = 0.2;
    goal.trajectory.points[index].positions[1] = 0.0;
    goal.trajectory.points[index].positions[2] = -1.5;
    goal.trajectory.points[index].positions[3] = 1.94;
    goal.trajectory.points[index].positions[4] = -1.57;
    goal.trajectory.points[index].positions[5] = -0.5;
    // Velocities
    goal.trajectory.points[index].velocities.resize(6);
    for (int j = 0; j < 6; ++j)
    {
      goal.trajectory.points[index].velocities[j] = 1.0;
    }
    // To be reached 2 second after starting along the trajectory
    goal.trajectory.points[index].time_from_start = ros::Duration(2.0);

    // Second trajectory point
    // Positions
    index += 1;
    goal.trajectory.points[index].positions.resize(6);
    goal.trajectory.points[index].positions[0] = 2.5;
    goal.trajectory.points[index].positions[1] = 0.2;
    goal.trajectory.points[index].positions[2] = -2.1;
    goal.trajectory.points[index].positions[3] = 1.9;
    goal.trajectory.points[index].positions[4] = 1.0;
    goal.trajectory.points[index].positions[5] = -0.5;
    // Velocities
    goal.trajectory.points[index].velocities.resize(6);
    for (int j = 0; j < 6; ++j)
    {
      goal.trajectory.points[index].velocities[j] = 0.0;
    }
    // To be reached 4 seconds after starting along the trajectory
    goal.trajectory.points[index].time_from_start = ros::Duration(4.0);

    ROS_INFO("End Waypoint");
  }


  
  void ArmTrajController::jointstatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {

    this->current_joint_state.header = msg->header;
    this->current_joint_state.name = msg->name;
    this->current_joint_state.position = msg->position;
    this->current_joint_state.velocity = msg->velocity;
    this->current_joint_state.effort = msg->effort;

    // for (auto &names_x :  this->current_joint_state.name) {
    //   ROS_WARN("Check names_x==%s", names_x.c_str());
    // }

    // for (auto &pos_x : this->current_joint_state.position) {
    //   ROS_WARN("Check Pos Values==%f", pos_x);
    // }

  }



  void ArmTrajController::goalposCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    /**
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    string[] name
    float64[] position
    float64[] velocity
    float64[] effort

    **/

    // We fill Data
    control_msgs::FollowJointTrajectoryGoal goal;

    ROS_INFO("Start Waypoint goalposCallback");

    // The joint names, which apply to all waypoints
    // for (auto &model_name_x : msg->name) {
    //   goal.trajectory.joint_names.push_back(model_name_x.c_str());
    // }

    // We force nly the 6 Joints
    for (int i = 0; i < 7; i++) {
      goal.trajectory.joint_names.push_back(msg->name[i].c_str());
    }

    for (auto &names_x :  goal.trajectory.joint_names) {
      ROS_WARN("Check names_x==%s", names_x.c_str());
    }


    // Two waypoints in this goal trajectory
    goal.trajectory.points.resize(1);

    // First trajectory point
    // Positions
    int index = 0;
    // int pos_num = 0;

    
    // Accces to Joint Positions
    // for (auto &value_pos_x : msg->position) {
    //   goal.trajectory.points[index].positions.push_back(value_pos_x);
    //   goal.trajectory.points[index].velocities.push_back(1.0);
    //   pos_num += 1;
    // }

    // We force Only the 6 Joints
    ROS_WARN("Using only 7 Joints==");
    for (int i = 0; i < 7; i++) {
      goal.trajectory.points[index].positions.push_back(msg->position[i]);
      goal.trajectory.points[index].velocities.push_back(1.0);
    }

    for (auto &pos_x : goal.trajectory.points[index].positions) {
      ROS_WARN("Check Pos Values==%f", pos_x);
    }

    for (auto &vel_x : goal.trajectory.points[index].velocities) {
      ROS_WARN("Check Vel Values==%f", vel_x);
    }

    // To be reached 2 second after starting along the trajectory
    goal.trajectory.points[index].time_from_start = ros::Duration(2.0);

    ROS_INFO("End Waypoint");

    // Sends the command to start the given trajectory 1s from now
    goal.trajectory.header.stamp = ros::Time::now(); // + ros::Duration(1.0);
    this->ArmClient->sendGoal(goal);


  }

  
  bool ArmTrajController::jointcommandServiceCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
         dynamixel_workbench_msgs::JointCommand::Response &res)
  {
    /**
    string unit  # rad ( for radians )
    uint8 id
    float32 goal_position
    ---
    bool result

    Example:
      joint_cmd_req.unit = "rad"
      joint_cmd_req.id = 1
      joint_cmd_req.goal_position = 0.7

    
    JointSate

    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    string[] name
    float64[] position
    float64[] velocity
    float64[] effort


    **/

    // create the joint trajectory based on the current joints state and the mod the joint given

    res.result = true;
    ROS_INFO("request: UNITS=%s, ID=%i, Value=%f", req.unit.c_str(), req.id, req.goal_position);


    // We fill Data
    control_msgs::FollowJointTrajectoryGoal goal;

    // We get data form JointState variable
    


    ROS_INFO("REV Start Waypoint jointcommandServiceCallback");

    // The joint names, which apply to all waypoints
    for (auto &model_name_x : this->current_joint_state.name) {
      goal.trajectory.joint_names.push_back(model_name_x.c_str());
    }

    for (auto &names_x :  goal.trajectory.joint_names) {
      ROS_WARN("Check names_x==%s", names_x.c_str());
    }


    // Two waypoints in this goal trajectory
    goal.trajectory.points.resize(1);

    // First trajectory point
    // Positions
    int index = 0;
    int pos_num = 1;

    
    // Accces to Joint Positions
    for (auto &value_pos_x : this->current_joint_state.position) {
      // We check if the Position Number Joint is the one asked for.
      if ( pos_num == req.id)
      {
        // Is so we change the vale from the joint state to the one asked for
        ROS_WARN("Joint to be changed==%i", pos_num);
        goal.trajectory.points[index].positions.push_back(req.goal_position);
      }else{
        ROS_WARN("NOT Joint to be changed==%i, %i", pos_num, req.id);
        goal.trajectory.points[index].positions.push_back(value_pos_x);
      }
      
      goal.trajectory.points[index].velocities.push_back(1.0);
      pos_num += 1;
    }

    for (auto &pos_x : goal.trajectory.points[index].positions) {
      ROS_WARN("Check Pos Values==%f", pos_x);
    }

    for (auto &vel_x : goal.trajectory.points[index].velocities) {
      ROS_WARN("Check Vel Values==%f", vel_x);
    }

    // To be reached 2 second after starting along the trajectory
    goal.trajectory.points[index].time_from_start = ros::Duration(2.0);

    ROS_INFO("End Waypoint");

    // Sends the command to start the given trajectory 1s from now
    goal.trajectory.header.stamp = ros::Time::now(); //+ ros::Duration(1.0);
    this->ArmClient->sendGoal(goal);


    return true;
  }


  void ArmTrajController::run()
  {
    ros::Rate loopRate(_rate);
    double halfPeriod = 0.5*1.0/_rate;

    _goalposSub = _nh.subscribe("goal_dynamixel_position", 1, &ArmTrajController::goalposCallback, this);

    _jointstatesSub = _nh.subscribe("joint_states", 1, &ArmTrajController::jointstatesCallback, this);


    _joint_command_service = _nh.advertiseService("joint_command", 
                                                  &ArmTrajController::jointcommandServiceCallback,
                                                  this);

    // while(!(actionClient->getState().isDone()) && ros::ok())
    while(ros::ok())
    {
      // ROS_INFO("In Run...");

      //check for subscriber's callbacks
      _cbQueue.callAvailable(ros::WallDuration(halfPeriod));

      loopRate.sleep();
    }

    ROS_INFO("Run Finished");
  }

  void ArmTrajController::stop()
  {
    ROS_DEBUG("Stopping");
    _goalposSub.shutdown();
  }


} //open_manipulator


int main(int argc, char**argv)
{
  ros::init (argc, argv, "open_manipulator_joint_trajectory_control");

  ros::NodeHandle nh, pnh("~");

  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  open_manipulator::ArmTrajController arm_controller_object(nh, pnh);

  // Create an arm controller action client to move the TIAGo's arm
  arm_controller_object.run();


  

  return EXIT_SUCCESS;

}
