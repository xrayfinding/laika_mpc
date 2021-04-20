#pragma once
// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/robot_hw.h>
#include "hardware_interface/imu_sensor_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
//#include "robot_state_interface.hpp"
#include <balance_controller/ros_controler/robot_state_interface.hpp>
#include "transmission_interface/transmission_info.h"
#include <transmission_interface/transmission_parser.h>
#include "laikago_controller/laikago_control_tool.h"
#include "laikago_msgs/MotorCmd.h"
#include "laikago_msgs/MotorState.h"

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>

// URDF
#include <urdf/model.h>

// Laikago SDK
#include <laikago_sdk/laikago_sdk.hpp>
#include <laikago_msgs/LowCmd.h>
#include <laikago_msgs/LowState.h>
#include <laikago_msgs/HighState.h>
#include <laikago_msgs/HighCmd.h>

#include "eigen3/Eigen/Eigen"
#include "unordered_map"

#include <fstream>
#include <iostream>

#include<algorithm>
#include "ros/package.h"
#include <cstring>
#include <string.h>
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "ros/advertise_service_options.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"

namespace laikago_ros_control {

class RobotStateLcmHardwareInterface : public hardware_interface::RobotHW
{

public:
  RobotStateLcmHardwareInterface();
  ~RobotStateLcmHardwareInterface();
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);

  void read(const ros::Time& time, const ros::Duration& period);

  void write(const ros::Time& time, const ros::Duration& period);

  bool Position_initCB(std_srvs::SetBool::Request& request,
                       std_srvs::SetBool::Response& response);

  bool Position_stopCB(std_srvs::SetBool::Request& request,
                       std_srvs::SetBool::Response& response);
  bool Controller_switchCB(controller_manager_msgs::SwitchController::Request& request,
                       controller_manager_msgs::SwitchController::Response& response);

  bool Init_flag();
  void update_loop();

  bool loadParameters(ros::NodeHandle& nh);

  //Convert laikago Imu data into sensor_msgs::IMU data
  const sensor_msgs::Imu getImuMsgs(laikago_msgs::LowState& low_state);

  void getFootForces(laikago_msgs::LowState& low_state);
  const sensor_msgs::JointState getJointStateMsgs(laikago_msgs::LowState& lowstate);


  void positionLimits(double &position){
      if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
          clamp(position, joint_urdf->limits->lower, joint_urdf->limits->upper);
  };
  void velocityLimits(double &velocity){
      if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
          clamp(velocity, -joint_urdf->limits->velocity, joint_urdf->limits->velocity);
  };
  void HAAeffortLimits(double &effort){
      if(effort>20){
          effort = 20;
      }else if(effort<-20){
          effort = -20;
      }
  };
  void HFEeffortLimits(double &effort){
      if(effort>30){
          effort = 30;
      }else if(effort<-30){
          effort = -30;
      }
  };
  void KFEeffortLimits(double &effort){
      if(effort>30){
          effort = 30;
      }else if(effort<-30){
          effort = -30;
      }
  };


  //bool loadParameters(ros::NodeHandle& nh);
  //bool checkPositionLimits(const int index, const double position);

protected:

  ros::NodeHandle node_handle_;

  std::vector<double> motor_friction_mu, motor_friction_bias,
                      motor_zero_offsets, motor_friction_proportion,
                      motor_max_limits, motor_min_limits, motor_directions;
  // Methods used to control a joint.
  enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID, STANCE_LEG, FREEZE};

  void registerJointLimits(const std::string& joint_name,
                           const hardware_interface::JointHandle& joint_handle,
                           const ControlMethod ctrl_method,
                           const ros::NodeHandle& joint_limit_nh,
                           const urdf::Model *const urdf_model,
                           int *const joint_type, double *const lower_limit,
                           double *const upper_limit, double *const effort_limit);
  unsigned int n_dof_;

  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;
  hardware_interface::ImuSensorInterface imu_interface_;
  hardware_interface::ImuSensorHandle::Data imu_data_;

  hardware_interface::RobotStateInterface robot_state_interface_;

  hardware_interface::RobotStateHandle::Data robot_state_data_;

  joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
  joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;

  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_effort_limits_;
  std::vector<ControlMethod> joint_control_methods_, last_joint_control_methods_;
  std::vector<control_toolbox::Pid> pid_controllers_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_effort_command_;
  std::vector<double> joint_position_command_;
  std::vector<double> last_joint_position_command_;
  std::vector<double> joint_velocity_command_;

  std::string physics_type_;

  // e_stop_active_ is true if the emergency stop is active.
  bool e_stop_active_, last_e_stop_active_;
  std::vector<bool> motor_disabled, motor_unused;

  double pos_read[12], pos_write[12], vel_read[12], vel_write[12], eff_read[12],eff_write[12];
  double position[3], orinetation[4], linear_vel[3], angular_vel[3],contact_pressure[4];
  int foot_contact[4],motor_status_word[12],mode_of_joint[12];

  std::vector<transmission_interface::TransmissionInfo> transmissions_;

  boost::recursive_mutex r_mutex_;
  boost::mutex mutex;
  boost::thread update_thread_;

private:

  //Laikago sdk
  laikago_msgs::LowCmd SendLowROS;
  laikago_msgs::LowState RecvLowROS;
  laikago_msgs::HighCmd SendHighROS;
  laikago_msgs::HighState RecvHighROS;
  laikago::LowCmd SendLowLCM;
  laikago::LowState RecvLowLCM;
  laikago::HighCmd SendHighLCM;
  laikago::HighState RecvHighLCM;
  laikago::LCM roslcm;
  geometry_msgs::WrenchStamped lf_contact_force,rf_contact_force,rh_contact_force,lh_contact_force;
  sensor_msgs::Imu imu_msgs_;
  sensor_msgs::JointState joint_state_msgs_;
  ServoCmd servoCmd;
  //MXR::NOTE: container to save multi_command
  std::vector<ServoCmd> multi_servoCmd;
  laikago_msgs::MotorState lastState;
  std::vector<laikago_msgs::MotorState> multi_lastState;
  std::vector<double> torque_set;

  //Publish IMU data
  ros::Publisher Imu_data_pub_;
  ros::Publisher joint_state_pub_;
  ros::Publisher lf_foot_contact_force_pub,rf_foot_contact_force_pub,
  rh_foot_contact_force_pub,lh_foot_contact_force_pub;
  std::string imu_topic_name_;

  urdf::JointConstSharedPtr joint_urdf;

  ros::ServiceServer laikago_position_init_server_,laikago_position_init_stop_server_,laikago_controller_switch_server_;
  bool init_flag,test_flag,laikago_position_init_buffer_,last_laikago_position_init_buffer_,laikago_position_init_stop_buffer_;
  std::string controller_name;
};


typedef boost::shared_ptr<RobotStateLcmHardwareInterface> RobotStateLcmHardwareInterfacePtr;

};
