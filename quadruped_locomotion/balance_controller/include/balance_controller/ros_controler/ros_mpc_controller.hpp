/*
 *  ros_balance_controller.hpp
 *  Descriotion: ros_controller use MPC method to control base pose
 *  TODO: Find the ways to our robot which has a heavy leg.
 *
 *  Created on: Feb 4, 2021
 *  Author: Yufeng Xu
 *  Institute: Harbin Institute of Technology, Shenzhen
 */
#pragma once

#include "controller_interface/controller.h"
#include "hardware_interface/joint_command_interface.h"
#include "balance_controller/contact_force_distribution/ContactForceDistribution.hpp"
#include "balance_controller/contact_force_distribution/ContactForceDistributionBase.hpp"
#include "balance_controller/contact_force_distribution/mpc_forcedistribution.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "balance_controller/ros_controler/robot_state_interface.hpp"
#include "balance_controller/ros_controler/gazebo_state_hardware_interface.hpp"
#include "single_leg_test/model_test_header.hpp"
#include <control_toolbox/pid.h>
#include <pluginlib/class_list_macros.hpp>
#include "free_gait_msgs/RobotState.h"
#include "sim_assiants/FootContacts.h"
#include "std_srvs/Empty.h"
#include "kindr_ros/kindr_ros.hpp"
#include "state_switcher/StateSwitcher.hpp"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Time.h"
#include "Eigen/Dense"
#include "Eigen/LU"
#include "tinyxml.h"

namespace balance_controller {
class RosMpcController : public controller_interface::Controller<hardware_interface::RobotStateInterface>
{
    typedef std::unordered_map<free_gait::LimbEnum, std::unique_ptr<StateSwitcher>, EnumClassHash> LimbState;
    typedef std::unordered_map<free_gait::LimbEnum, ros::Time, EnumClassHash> LimbDuration;
    typedef std::unordered_map<free_gait::LimbEnum, bool, EnumClassHash> LimbFlag;
    typedef std::unordered_map<free_gait::LimbEnum, double, EnumClassHash> LimbPhase;
    typedef std::unordered_map<free_gait::LimbEnum, free_gait::Vector, EnumClassHash> LimbVector;

public:
    RosMpcController();
    ~RosMpcController();
    /**
     * @brief init
     * @param hardware
     * @param node_handle
     * @return
     */
    bool init(hardware_interface::RobotStateInterface* hardware,
              ros::NodeHandle& node_handle);
    void update(const ros::Time& time, const ros::Duration& period);
    void starting(const ros::Time& time);
    void stopping(const ros::Time& time);
    /**
     * @brief joint_names
     */
    std::vector<std::string> joint_names;
    /**
     * @brief joints, a vector of joint handle, handle the joint of hardware
     * interface
     */
    std::vector<hardware_interface::JointHandle> joints;
    std::vector<hardware_interface::JointHandle> position_joints;
//    std::vector<hardware_interface::RobotStateHandle> joints;
    /**
     * @brief robot_state_handle, handle robot state
     */
    hardware_interface::RobotStateHandle robot_state_handle;
    /**
     * @brief commands_buffer,TODO
     */
    realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer;
    realtime_tools::RealtimeBuffer<Pose> command_pose_buffer;
    realtime_tools::RealtimeBuffer<LimbVector> command_foot_buffer, command_foot_vel_buffer;
    unsigned int n_joints;


    struct LegInfo
    {
      bool isPartOfForceDistribution_;
      bool isLoadConstraintActive_;
      int indexInStanceLegList_;
      int startIndexInVectorX_;
      Force desiredContactForce_;
      //! for logging
      Vector firstDirectionOfFrictionPyramidInWorldFrame_;
      //! for logging
      Vector secondDirectionOfFrictionPyramidInWorldFrame_;
      //! for logging
      Vector normalDirectionOfFrictionPyramidInWorldFrame_;
      //! Assumed friction coefficient (mu).
      double frictionCoefficient_;
      double loadFactor_;

    };

    std::map<free_gait::LimbEnum, LegInfo> legInfos_;
    ros::Time time_test;
 private:
    /**
     * @brief base_command_sub_,subscribe base_command and contact information
     */
    ros::Subscriber base_command_sub_, contact_sub_;
    /**
     * @brief robot_state_, State class to save all the robot state ,and provide method of
     * kinemaics
     */
    std::shared_ptr<free_gait::State> robot_state_;
    std::shared_ptr<free_gait::State> robot_state;
    Position base_desired_position;
    RotationQuaternion base_desired_rotation;
    LinearVelocity base_desired_linear_velocity;
    LocalAngularVelocity base_desired_angular_velocity;

    std::vector<free_gait::LimbEnum> limbs_;
    std::vector<free_gait::BranchEnum> branches_;

    LimbState limbs_state, limbs_desired_state, limbs_last_state;
    LimbFlag real_contact_, is_cartisian_motion_, is_footstep_, is_legmode_;
    LimbDuration t_sw0, t_st0;
    LimbFlag sw_flag, st_flag;
    LimbPhase sw_phase, st_phase;
    LimbVector foot_positions, foot_velocities, foot_accelerations, real_contact_force_, stored_foot_positions;
    /*
     * The cosnt parameter mpc_controller
     */

    //****************************************************
    //roll-pitch-yaw x-y-z ang-vel vel gravity-place-holder
    //****************************************************

    std::vector<double> inertia_list_mpc = {0.07335,0,0, 0,0.25068,0, 0,0,0.25447};
    //Golaoxu : the follor weights has been test . and it is ok.
    //std::vector<double> _MPC_WEIGHTS = {5, 5, 0.2, 3, 3, 50, 0., 0., 1., 1., 1., 10., 0};
    //Golaoxu : the follor weights has been test . and it is ok.
    //std::vector<double> _MPC_WEIGHTS = {5, 10, 5, 3, 3, 50, 0., 0., 10., 1., 1., 10., 0};

    //Golaoxu: in MIT's ways
    //TODO:
    //test:*********************************************
    //std::vector<double> _MPC_WEIGHTS = {5, 5, 0.2, 3, 3, 30, 0., 0., 10., 1., 1., 10., 0};
    //in mit cheetah-software
    //std::vector<double> _MPC_WEIGHTS = {0.25, 0.25, 10, 2, 2, 20, 0., 0., 0.3, 0.2, 0.2, 0.2, 0};
    std::vector<double> _MPC_WEIGHTS;
    //in ros_balance_controller the weights is following and its ok
    //std::vector<double> _MPC_WEIGHTS = {20, 20, 5, 10, 10, 100, 0., 0., 1., 1., 1., 1., 0};

    /**
     * @brief contact_distribution_ , pointer to contact force optimaziton/or use mpc to conpute the contact force
     */
    std::shared_ptr<ContactForceDistribution> contact_distribution_;
    std::shared_ptr<MPC::ConvexMpc> mpc_solver;
       /**
     * @brief virtual_model_controller_, pointer to virtual model controller
     */

    std::shared_ptr<VirtualModelController> virtual_model_controller_;

    std::shared_ptr<MyRobotSolver> single_leg_solver_;

    std::vector<control_toolbox::Pid> pid_controllers_;       /**< Internal PID controllers. */

    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;
    /**
     * @brief baseCommandCallback, ros subscriber callback
     * @param robot_state
     */
    void baseCommandCallback(const free_gait_msgs::RobotStateConstPtr& robot_state_msg);
    void footContactsCallback(const sim_assiants::FootContactsConstPtr& foot_contacts);

    void enforceJointLimits(double &command, unsigned int index);
    double computeTorqueFromPositionCommand(double command, int i, const ros::Duration& period);

    void contactStateMachine();
    bool jointTorquesLimit(free_gait::JointEffortsLeg& joint_torque, double max_torque);
    /**
     * @brief r_mutex_
     */
    boost::recursive_mutex r_mutex_;
    /**
     * @brief joint_command_pub_, for debug to monitor
     */
    ros::Publisher joint_command_pub_, base_command_pub_, base_actual_pub_, joint_actual_pub_,
    leg_state_pub_, contact_desired_pub_, leg_phase_pub_, desired_robot_state_pub_, actual_robot_state_pub_,
    motor_status_word_pub_, vmc_info_pub_, desired_vmc_info_pub_;
    std::vector<nav_msgs::Odometry> base_command_pose_, base_actual_pose_;
    std::vector<sensor_msgs::JointState> joint_command_, joint_actual_;
    std::vector<std_msgs::Int8MultiArray> leg_states_;
    std::vector<sim_assiants::FootContacts> foot_desired_contact_;
    std::vector<std_msgs::Float64MultiArray> leg_phases_;
    std::vector<free_gait_msgs::RobotState> desired_robot_state_, actual_robot_state_;
    std::vector<geometry_msgs::WrenchStamped> vitual_force_torque_, desired_vitual_force_torque_;
    std::vector<std_msgs::Time> log_time_;
    std::vector<std_msgs::Int8MultiArray> motor_status_word_;
    ros::ServiceServer log_data_srv_;

    int log_length_, log_index_;
    bool logDataCapture(std_srvs::Empty::Request& req,
                        std_srvs::Empty::Response& res);
    LimbFlag update_surface_normal_flag, store_current_joint_state_flag_, stored_current_foot_position_flag;
//    bool store_current_joint_state_flag_;
    LimbVector surface_normals;
//    std::vector<double> stored_limb_joint_position_;
    free_gait::JointPositions stored_limb_joint_position_;

    int delay_counts[4];

    bool real_robot, ignore_contact_sensor,log_data;

    double initial_pressure[4];
    double contact_pressure_bias;

    /*
     *GoLaoxu:
     * 1. func and
     * 2. variables
     *  for mpc controller.
     *
     */
    bool collections_4_mpc();
    bool collections_4_mpc(bool ispybulletways);
    bool collections_4_mpc(int is_mpc_use);
    bool computeJointTorques(std::vector<double>& _forces);
    bool QuaternionToEuler_desired();
    bool QuaternionToEuler();
    bool loadParams_MPC(const ros::NodeHandle& node_handle);

    int steps_MPC;
    double delta_t_MPC;
    double torque_weight;
    std::vector<double> com_position;
    std::vector<double> com_velocity;
    std::vector<double> com_roll_pitch_yaw;
    std::vector<double> com_angular_velocity;
    std::vector<int> foot_contact_states;
    std::vector<double> foot_positions_body_frame;
    std::vector<double> foot_friction_coeffs;
    std::vector<double> desired_com_position;
    std::vector<double> desired_com_velocity;
    std::vector<double> desired_com_roll_pitch_yaw;
    std::vector<double> desired_com_angular_velocity;

    std::vector<double> desired_quaternion;
    std::vector<double> quaternion;
    std::queue<vector<double>> control_steps_mpc;
};

}
