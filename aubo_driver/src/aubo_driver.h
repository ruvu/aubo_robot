#ifndef AUBO_DRIVER_H_
#define AUBO_DRIVER_H_

// AUBO IO
#include "serviceinterface.h"

// ROS IO
#include <ros/publisher.h>
#include <ros/timer.h>

#include <ros/service_server.h>
#include <aubo_msgs/SetIO.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#define ARM_DOF 6

namespace aubo_driver
{
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> ActionServerType;

class AuboDriver
{
public:
  AuboDriver(std::string ip, int port, double rate, double io_flag_delay);
  ~AuboDriver();

private:
  // Aubo robot interfaces
  ServiceInterface robot_service_;

  //
  ros::Publisher joint_state_publisher_;
  ros::Publisher robot_status_publisher_;
  ros::Publisher io_publisher_;

  std::shared_ptr<ActionServerType> action_server_;
  void executeCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);

  void timerCallback(const ros::TimerEvent& e);
  void publishIOMsg();

  ros::Timer get_robot_state_timer_;

  ros::ServiceServer io_srv_;
  bool setIO(aubo_msgs::SetIORequest& req, aubo_msgs::SetIOResponse& resp);

  ros::Duration io_flag_delay_;
};

}

#endif /* AUBO_DRIVER_H_ */
