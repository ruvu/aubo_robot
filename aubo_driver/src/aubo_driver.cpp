#include "aubo_driver.h"

#include <sensor_msgs/JointState.h>
#include <industrial_msgs/RobotStatus.h>

namespace aubo_driver {

const std::string JOINT_NAMES[ARM_DOF] = {"shoulder_joint", "upperArm_joint", "foreArm_joint",
                                          "wrist1_joint", "wrist2_joint", "wrist3_joint"};

AuboDriver::AuboDriver(std::string ip, int port, double rate)
{
  ROS_INFO("Connecting to %s:%d", ip.c_str(), port);

  // Set-up Aubo IO
  int return_code = robot_service_.robotServiceLogin(ip.c_str(), port, "aubo", "123456");
  if(return_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    std::runtime_error("Failed to connect to robot on " + ip + ":" + std::to_string(port));
  }

  // Set-up ROS IO
  ros::NodeHandle nh;
  action_server_ = std::shared_ptr<ActionServerType>(
        new ActionServerType(nh, "arm_controller/follow_joint_trajectory",
                             boost::bind(&AuboDriver::executeCallback, this, _1), false));
  joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>("joint_states", 300);
  robot_status_publisher_ = nh.advertise<industrial_msgs::RobotStatus>("robot_status", 100);
  io_srv_ = nh.advertiseService("set_io",&AuboDriver::setIO, this);
  action_server_->start();

  // Obtain the robot state at desired rate
  get_robot_state_timer_ = nh.createTimer(ros::Rate(rate).expectedCycleTime(), &AuboDriver::timerCallback, this);
  get_robot_state_timer_.start();
}

AuboDriver::~AuboDriver()
{
  /** log out　**/
  robot_service_.robotServiceLogout();
}

void AuboDriver::executeCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
  ROS_INFO("Received goal!");
  if (goal->trajectory.points.size() == 0)
  {
    ROS_ERROR("No points in traj");
    action_server_->setAborted();
    return;
  }
  const trajectory_msgs::JointTrajectoryPoint& point = goal->trajectory.points.back();
  if (point.positions.size() != ARM_DOF || goal->trajectory.joint_names.size() != ARM_DOF) {
    ROS_ERROR("Trajectort joint names and positions per point should be of length 6");
    action_server_->setAborted();
    return;
  }

  double targetPoint[ARM_DOF];
  for (size_t i = 0; i < ARM_DOF; ++i)
  {
    bool found = false;
    for (size_t j = 0; j < ARM_DOF; ++j)
    {
      if (JOINT_NAMES[i] == goal->trajectory.joint_names[j])
      {
        targetPoint[i] = point.positions[j];
        found = true;
        break;
      }
    }
    if (!found) {
      ROS_ERROR("Invalid joint name in joint_names");
      action_server_->setAborted( );
    }
  }

  control_msgs::FollowJointTrajectoryResult result;
  result.error_code = robot_service_.robotServiceJointMove(targetPoint, true);
  ROS_INFO("Move to the goal returned %d", result.error_code);

  if (result.error_code == aubo_robot_namespace::InterfaceCallSuccCode)
  {
    action_server_->setSucceeded(result);
  }
  else
  {
    action_server_->setAborted(result);
  }
}

void AuboDriver::timerCallback(const ros::TimerEvent& e)
{
  {
    // Obtain joint status
    aubo_robot_namespace::JointStatus joint_status[ARM_DOF];
    if (robot_service_.robotServiceGetRobotJointStatus(joint_status, ARM_DOF) != aubo_robot_namespace::InterfaceCallSuccCode) {
      throw std::runtime_error("Failed to obtain joint state");
    }

    // Publish the joint state message
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.name.resize(ARM_DOF);
    joint_state_msg.position.resize(ARM_DOF);
    for(size_t i = 0; i < ARM_DOF; i++)
    {
      joint_state_msg.name[i] = JOINT_NAMES[i];
      joint_state_msg.position[i] = joint_status[i].jointPosJ;
    }
    joint_state_publisher_.publish(joint_state_msg);
  }

  {
    // Obtain robot status
    aubo_robot_namespace::RobotState robot_state;
    if (robot_service_.robotServiceGetRobotCurrentState(robot_state) != aubo_robot_namespace::InterfaceCallSuccCode) {
      throw std::runtime_error("Failed to obtain robot state");
    }

    // Publish robot status
    industrial_msgs::RobotStatus robot_status_msg;
    robot_status_msg.motion_possible.val = (int8) robot_state;
    robot_status_msg.in_motion.val       = (int8) robot_state;
    robot_status_publisher_.publish(robot_status_msg);
  }
}

#define USER_DO_PIN_OFFSET 32
bool AuboDriver::setIO(aubo_msgs::SetIORequest& req, aubo_msgs::SetIOResponse& resp)
{
  if (req.fun == 1) // Digital out
  {
    resp.success = !robot_service_.robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDO,
                                                                req.pin + USER_DO_PIN_OFFSET, req.state);
  }
  else if (req.fun == 2) // Analogue out
  {
    resp.success = !robot_service_.robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotBoardUserAO,
                                                                req.pin, req.state);
  }
  else
  {
    resp.success = false;
  }
  return resp.success;
}

}

using namespace aubo_driver;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "aubo_driver");

  ros::NodeHandle local_nh("~");
  try
  {
    AuboDriver driver(local_nh.param("ip", std::string("192.168.1.34")),
                      local_nh.param("port", 8899),
                      local_nh.param("rate", 50));
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM("AUBO hardware error: " << e.what());
  }

  ROS_INFO("Exiting aubo_driver ...");

  return 0;
}


