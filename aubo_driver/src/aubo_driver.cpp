#include "aubo_driver/aubo_driver.h"

#include <string>
#include <sys/timeb.h>

namespace aubo_driver {

const std::string JOINT_NAMES[ARM_DOF] = {"shoulder_joint","upperArm_joint","foreArm_joint","wrist1_joint","wrist2_joint","wrist3_joint"};

AuboDriver::AuboDriver(std::string ip, int port, double io_flag_delay) :
  io_flag_delay_(io_flag_delay)
{
  ros::NodeHandle nh;

  action_server_ = std::shared_ptr<ActionServerType>(
        new ActionServerType(nh, "arm_controller/follow_joint_trajectory", boost::bind(&AuboDriver::executeCallback, this, _1), false));
  joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>("joint_states", 300);
  robot_status_publisher_ = nh.advertise<industrial_msgs::RobotStatus>("robot_status", 100);
  io_publisher_ = nh.advertise<aubo_msgs::IOState>("io_state", 1);
  io_srv_ = nh.advertiseService("set_io",&AuboDriver::setIO, this);

    action_server_->start();

    ROS_INFO("Start the driver!");
    int ret1 = aubo_robot_namespace::InterfaceCallSuccCode;
    int ret2 = aubo_robot_namespace::InterfaceCallSuccCode;

    ROS_INFO("Connecting to %s:%d", ip.c_str(), port);

    if(!robot_service_.robotServiceLogin(ip.c_str(), port, "aubo", "123456") == aubo_robot_namespace::InterfaceCallSuccCode)
    {
      std::runtime_error("Failed to connect to robot on " + ip + ":" + std::to_string(port));
    }

    //communication Timer between ros node and real robot controller.
    timer = nh.createTimer(ros::Duration(0.020), &AuboDriver::timerCallback,this);
    timer.start();
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
      if (JOINT_NAMES[i] == goal->trajectory.joint_names[j]) {
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
          aubo_robot_namespace::JointStatus jointStatus[ARM_DOF];
          aubo_robot_namespace::RobotDiagnosis robotDiagnosisInfo;
          aubo_robot_namespace::RobotState state;
          aubo_robot_namespace::RobotErrorCode code;

        /** 接口调用: 获取关节状态 **/
        int ret = robot_service_.robotServiceGetRobotJointStatus(jointStatus, 6);
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            double joints[] = {jointStatus[0].jointPosJ,jointStatus[1].jointPosJ,jointStatus[2].jointPosJ,jointStatus[3].jointPosJ,jointStatus[4].jointPosJ,jointStatus[5].jointPosJ};
            sensor_msgs::JointState joint_state;
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(ARM_DOF);
            joint_state.position.resize(ARM_DOF);
            for(int i = 0; i<ARM_DOF; i++)
            {
                joint_state.name[i] = JOINT_NAMES[i];
                joint_state.position[i] = joints[i];
            }
            joint_state_publisher_.publish(joint_state);
        }
//        Get the buff size of thr rib


        ret = robot_service_.robotServiceGetRobotCurrentState(state);
//        robot_service_.getErrDescByCode(rs.code);

        // pub robot_status information to the controller action server.
        industrial_msgs::RobotStatus robotstatus;

        robotstatus.mode.val            = (int8)robotDiagnosisInfo.orpeStatus;
        robotstatus.e_stopped.val       = (int8)robotDiagnosisInfo.softEmergency;
        robotstatus.drives_powered.val  = (int8)robotDiagnosisInfo.armPowerStatus;
        robotstatus.motion_possible.val = (int8)state;
        robotstatus.in_motion.val       = (int8)state;
//        robotstatus.in_error.val        = (int8)rs.code;
//        robotstatus.error_code          = (int32)rs.code;
        robot_status_publisher_.publish(robotstatus);

    publishIOMsg();


}

void AuboDriver::publishIOMsg()
{
    aubo_msgs::IOState io_msg;
    // robot control board IO
    std::vector<aubo_robot_namespace::RobotIoDesc> statusVectorIn;
    std::vector<aubo_robot_namespace::RobotIoDesc> statusVectorOut;
    std::vector<aubo_robot_namespace::RobotIoType>  ioTypeIn;
    std::vector<aubo_robot_namespace::RobotIoType>  ioTypeOut;
    ioTypeIn.push_back(aubo_robot_namespace::RobotBoardUserDI);
    ioTypeOut.push_back(aubo_robot_namespace::RobotBoardUserDO);
    int ret = robot_service_.robotServiceGetBoardIOStatus(ioTypeIn,statusVectorIn);
    ret = robot_service_.robotServiceGetBoardIOStatus(ioTypeOut,statusVectorOut);
    //F1-F6 are reserved.
    char num[2];
    for (unsigned int i = 6; i < statusVectorIn.size(); i++)
    {
        aubo_msgs::Digital digi;
        num[0] = statusVectorIn[i].ioName[5];
        num[1] = statusVectorIn[i].ioName[6];
        digi.pin = std::atoi(num);
//            digi.pin = statusVectorIn[i].ioAddr - 36;
        digi.state = statusVectorIn[i].ioValue;
        digi.flag = 0;
        io_msg.digital_in_states.push_back(digi);
    }
    for (unsigned int i = 0; i < statusVectorOut.size(); i++)
    {
        aubo_msgs::Digital digo;
        num[0] = statusVectorOut[i].ioName[5];
        num[1] = statusVectorOut[i].ioName[6];
        digo.pin = std::atoi(num);
        int addr = statusVectorOut[i].ioAddr;
//            digo.pin = statusVectorOut[i].ioAddr - 32;
        digo.state = statusVectorOut[i].ioValue;
        digo.flag = 1;
        io_msg.digital_out_states.push_back(digo);
    }

    statusVectorIn.clear();
    statusVectorOut.clear();
    ioTypeIn.clear();
    ioTypeOut.clear();
    ioTypeIn.push_back(aubo_robot_namespace::RobotBoardUserAI);
    ioTypeOut.push_back(aubo_robot_namespace::RobotBoardUserAO);
    ret = robot_service_.robotServiceGetBoardIOStatus(ioTypeIn,statusVectorIn);
    ret = robot_service_.robotServiceGetBoardIOStatus(ioTypeOut,statusVectorOut);
    for (unsigned int i = 0; i < statusVectorIn.size(); i++)
    {
        aubo_msgs::Analog ana;
        ana.pin = statusVectorIn[i].ioAddr;
        ana.state = statusVectorIn[i].ioValue;
        io_msg.analog_in_states.push_back(ana);
    }

    for (unsigned int i = 0; i < statusVectorOut.size(); i++)
    {
        aubo_msgs::Analog ana;
        ana.pin = statusVectorOut[i].ioAddr;
        ana.state = statusVectorOut[i].ioValue;
        io_msg.analog_out_states.push_back(ana);
    }

//        // robot tool IO
//        statusVectorIn.clear();
//        statusVectorOut.clear();
//        ret = robot_service_.robotServiceGetAllToolDigitalIOStatus(statusVectorIn);
//        ret = robot_service_.robotServiceGetAllToolAIStatus(statusVectorOut);
//        for (unsigned int i = 0; i < statusVectorIn.size(); i++)
//        {
//            aubo_msgs::Digital digo;
//            digo.pin = statusVectorIn[i].ioAddr;
//            digo.state = statusVectorIn[i].ioValue;
//            digo.flag = (statusVectorIn[i].ioType == aubo_robot_namespace::RobotToolDI)? 0 : 1;
//            io_msg.tool_io_states.push_back(digo);
//        }

    for (unsigned int i = 0; i < statusVectorOut.size(); i++)
    {
        aubo_msgs::Analog ana;
        ana.pin = statusVectorOut[i].ioAddr;
        ana.state = statusVectorOut[i].ioValue;
        io_msg.tool_ai_states.push_back(ana);
    }
    io_publisher_.publish(io_msg);
}

bool AuboDriver::setIO(aubo_msgs::SetIORequest& req, aubo_msgs::SetIOResponse& resp)
{
    resp.success = true;
    if (req.fun == 1)
    {
        robot_service_.robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDO,req.pin + 32, req.state);
        ros::Duration(io_flag_delay_).sleep();
    }
    else if (req.fun == 2)
    {
        robot_service_.robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotBoardUserAO,req.pin, req.state);
        ros::Duration(io_flag_delay_).sleep();
    }
    else if (req.fun == 3)
    {
        if(req.state == -1)
        {
            robot_service_.robotServiceSetToolDigitalIOType((aubo_robot_namespace::ToolDigitalIOAddr)req.pin, aubo_robot_namespace::IO_IN);
            ros::Duration(io_flag_delay_).sleep();
        }
        else
        {
            robot_service_.robotServiceSetToolDigitalIOType((aubo_robot_namespace::ToolDigitalIOAddr)req.pin, aubo_robot_namespace::IO_OUT);
            ros::Duration(io_flag_delay_).sleep();
            robot_service_.robotServiceSetToolDOStatus((aubo_robot_namespace::ToolDigitalIOAddr)req.pin, (aubo_robot_namespace::IO_STATUS)req.state);
            ros::Duration(io_flag_delay_).sleep();
        }
    }
    else if (req.fun == 4)
    {
        robot_service_.robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotToolAO,req.pin, req.state);
        ros::Duration(io_flag_delay_).sleep();
    }
    else if (req.fun == 5)
    {
        //0->0V; 1->12V; 2->24V
        robot_service_.robotServiceSetToolPowerVoltageType((aubo_robot_namespace::ToolPowerType) req.state);
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
    AuboDriver driver(local_nh.param("ip", std::string("192.168.1.34")),
                          local_nh.param("port", 8899),
                          local_nh.param("io_flag_delay", 0.02));

    ros::spin();

    ROS_WARN("Exiting aubo_driver ...");

    return 0;
}


