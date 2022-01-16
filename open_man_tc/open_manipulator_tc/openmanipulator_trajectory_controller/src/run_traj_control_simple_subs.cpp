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

    void run(arm_control_client_Ptr& actionClient);


  protected:

    void cloudCallback(const sensor_msgs::JointState::ConstPtr& msg);

    void start();
    void stop();

    ros::NodeHandle& _nh, _pnh;
    ros::CallbackQueue _cbQueue;

    

    double _rate;

    // ROS subscribers
    ros::Subscriber _cloudSub;

  };

  ArmTrajController::ArmTrajController(ros::NodeHandle& nh,
                                     ros::NodeHandle& pnh):
    _nh(nh),
    _pnh(pnh),
    _rate(0.2)
  {
    _nh.setCallbackQueue(&_cbQueue);

    pnh.param<double>("rate", _rate, _rate);

    ROS_INFO_STREAM("The node will operate at maximum " << _rate << " Hz");

  }

  ArmTrajController::~ArmTrajController()
  {
  }


  // Create a ROS action client to move TIAGo's arm
  void ArmTrajController::createArmClient(arm_control_client_Ptr& actionClient)
  {
    ROS_INFO("Creating action client to arm controller ...");

    actionClient.reset( new arm_control_client("/arm/follow_joint_trajectory") );

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


  void ArmTrajController::cloudCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    ROS_INFO("Joint State name: [%s]", msg->name[0]);    
  }

  void ArmTrajController::run(arm_control_client_Ptr& actionClient)
  {
    ros::Rate loopRate(_rate);
    double halfPeriod = 0.5*1.0/_rate;

    _cloudSub = _nh.subscribe("/goal_dynamixel_position", 1, &ArmTrajController::cloudCallback, this);

    // while(!(actionClient->getState().isDone()) && ros::ok())
    while(ros::ok())
    {
      ROS_INFO("In Run...");
      // Generates the goal for the TIAGo's arm
      control_msgs::FollowJointTrajectoryGoal arm_goal;
      this->waypoints_arm_goal(arm_goal);

      // Sends the command to start the given trajectory 1s from now
      arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
      actionClient->sendGoal(arm_goal);

      //check for subscriber's callbacks
      _cbQueue.callAvailable(ros::WallDuration(halfPeriod));

      loopRate.sleep();
    }

    ROS_INFO("Run Finished");
  }

  void ArmTrajController::stop()
  {
    ROS_DEBUG("Stopping");
    _cloudSub.shutdown();
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
  arm_control_client_Ptr ArmClient;
  arm_controller_object.createArmClient(ArmClient);


  arm_controller_object.run(ArmClient);

  return EXIT_SUCCESS;

}
