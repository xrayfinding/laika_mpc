/*
 *  ros_balance_controller.hpp
 *  Descriotion: ros_controller use MPC method to control base pose
 *  TODO: Find the ways to our robot which has a heavy leg.
 *
 *  Created on: Feb 4, 2021
 *  Author: Yufeng Xu
 *  Institute: Harbin Institute of Technology, Shenzhen
 */
#include "balance_controller/ros_controler/ros_balance_controller.hpp"
#include "balance_controller/ros_controler/ros_mpc_controller.hpp"
#include "controller_manager/controller_manager.h"
namespace balance_controller{
    RosMpcController::RosMpcController(){
        log_length_ = 10000;
        log_index_ = log_length_;
        limbs_.push_back(free_gait::LimbEnum::LF_LEG);
        limbs_.push_back(free_gait::LimbEnum::RF_LEG);
        limbs_.push_back(free_gait::LimbEnum::LH_LEG);
        limbs_.push_back(free_gait::LimbEnum::RH_LEG);

        branches_.push_back(free_gait::BranchEnum::BASE);
        branches_.push_back(free_gait::BranchEnum::LF_LEG);
        branches_.push_back(free_gait::BranchEnum::RF_LEG);
        branches_.push_back(free_gait::BranchEnum::LH_LEG);
        branches_.push_back(free_gait::BranchEnum::RH_LEG);
        //! WSHY: initialize STAte
        robot_state_.reset(new free_gait::State);
        robot_state_->initialize(limbs_, branches_);

        robot_state.reset(new free_gait::State);
        robot_state->initialize(limbs_, branches_);
        for(auto limb : limbs_)
          {
            limbs_state[limb].reset(new StateSwitcher);
            limbs_state.at(limb)->initialize(0);
            limbs_last_state[limb].reset(new StateSwitcher);
            limbs_last_state.at(limb)->initialize(0);
            limbs_desired_state[limb].reset(new StateSwitcher);
            limbs_desired_state.at(limb)->initialize(0);
            t_sw0[limb] = ros::Time::now();
            t_st0[limb] = ros::Time::now();
            sw_phase[limb] = 0.0;
            st_phase[limb] = 0.0;
            sw_flag[limb] = false;
            st_flag[limb] = false;
            surface_normals[limb] = Vector(0,0,1);
            foot_positions[limb] = Vector(0,0,0);
            foot_velocities[limb] = Vector(0,0,0);
            foot_accelerations[limb] = Vector(0,0,0);
            stored_foot_positions[limb] = Vector(0,0,0);
            stored_current_foot_position_flag[limb] = false;
            store_current_joint_state_flag_[limb] = false;
            update_surface_normal_flag[limb] = false;
            real_contact_[limb] = false;
            real_contact_force_[limb] = Vector(0,0,0);
            is_cartisian_motion_[limb] = false;
            is_footstep_[limb] = false;
            is_legmode_[limb] = false;
          }
        for(auto leg : limbs_) {
          legInfos_[leg] = LegInfo();
        }
        for(int i=0;i<4;i++)
          {
            initial_pressure[i] = 0;
          }
    };
    RosMpcController::~RosMpcController()
    {
      base_command_sub_.shutdown();
      contact_sub_.shutdown();
    };
    bool RosMpcController::init(hardware_interface::RobotStateInterface* hardware,
                                ros::NodeHandle& node_handle){
        ROS_INFO("Initializing RosMpcController");
        contact_distribution_.reset(new ContactForceDistribution(node_handle, robot_state));
        //mpc_solver.reset(new MPC::ConvexMpc(robot_state->getRobotMass(),inertia_list_mpc,4, 15, 0.05, _MPC_WEIGHTS));
        //golaoxu:following are ok
        //mpc_solver.reset(new MPC::ConvexMpc(robot_state->getRobotMass(),inertia_list_mpc,4, 4, 0.05, _MPC_WEIGHTS,1e-7));
        //before test
        //mpc_solver.reset(new MPC::ConvexMpc(robot_state->getRobotMass(),inertia_list_mpc,4, 10, 0.05, _MPC_WEIGHTS,5e-8));

        //in balance controller
        loadParams_MPC(node_handle);
        //mpc_solver.reset(new MPC::ConvexMpc(robot_state->getRobotMass() ,inertia_list_mpc,4, 10, delta_t_MPC, _MPC_WEIGHTS));
        mpc_solver.reset(new MPC::ConvexMpc(robot_state->getRobotMass() ,inertia_list_mpc,4, steps_MPC, delta_t_MPC, _MPC_WEIGHTS,torque_weight));
        single_leg_solver_.reset(new MyRobotSolver(node_handle, robot_state));
        single_leg_solver_->model_initialization();
        if(!single_leg_solver_->loadLimbModelFromURDF())
          {
            ROS_ERROR("Failed to load model from URDF file");
          }
        if(!single_leg_solver_->loadParameters())
          {
            ROS_INFO("SWC load parameters failed");
          }

        if(!node_handle.getParam("real_robot", real_robot))
          {
            ROS_ERROR("Can't find parameter of 'real_robot'");
            return false;
          }

        if(!node_handle.getParam("contact_pressure_bias", contact_pressure_bias))
          {
            ROS_ERROR("Can't find parameter of 'contact_pressure_bias'");
            return false;
          }

        if(!node_handle.getParam("ignore_contact_sensor", ignore_contact_sensor))
          {
            ROS_ERROR("Can't find parameter of 'ignore_contact_sensor'");
            return false;
          }
        if(!node_handle.getParam("log_data", log_data))
          {
            ROS_ERROR("Can't find parameter of 'log_data'");
            return false;
          }

        urdf::Model urdf;
        if (!urdf.initParam("/robot_description"))
        {
          ROS_ERROR("Failed to parse urdf file");
          return false;
        }

        //! WSHY: get joint handle from robot state handle
        std::string param_name = "joints";
        if(!node_handle.getParam(param_name, joint_names))
          {
            ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << node_handle.getNamespace() << ").");
            return false;
          }
        n_joints = joint_names.size();
        if(n_joints == 0){
              ROS_ERROR_STREAM("List of joint names is empty.");
              return false;
            }
        pid_controllers_.resize(n_joints);
        for(unsigned int i = 0; i < n_joints; i++)
          {
            try {
    //          joints.push_back(hardware->getHandle(joint_names[i]));
              joints.push_back(hardware->joint_effort_interfaces_.getHandle(joint_names[i]));
              position_joints.push_back(hardware->joint_position_interfaces_.getHandle(joint_names[i]));
              ROS_INFO("Get '%s' Handle", joint_names[i].c_str());
    //          hardware->g
            } catch (const hardware_interface::HardwareInterfaceException& ex) {
              ROS_ERROR_STREAM("Exception thrown : "<< ex.what());
              return false;
            }
            urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names[i]);
            if (!joint_urdf)
            {
              ROS_ERROR("Could not find joint '%s' in urdf", joint_names[i].c_str());
              return false;
            }
            joint_urdfs_.push_back(joint_urdf);

            // Load PID Controller using gains set on parameter server
            if (!pid_controllers_[i].init(ros::NodeHandle(node_handle, joint_names[i] + "/pid")))
            {
              ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names[i] + "/pid");
              return false;
            }


          }


        ROS_INFO("Mpc Controller to Get robot state handle");
        //! WSHY: get robot state handle
        robot_state_handle = hardware->getHandle("base_controller");

        ROS_INFO("Mpc Controller initialized");
        for(unsigned int i=0;i<12;i++)
          {
            robot_state_handle.getJointEffortWrite()[i] = 0;
            robot_state_handle.motor_status_word_[i] = 0;
          }
        for(int i = 0;i<4;i++)
          {
            robot_state_handle.foot_contact_[i] = 1;
            delay_counts[i] = 0;
          }
            for(int i =0;i<4;i++)
              {
                limbs_state.at(static_cast<free_gait::LimbEnum>(i))->setState(StateSwitcher::States::StanceNormal);
                robot_state->setSupportLeg(static_cast<free_gait::LimbEnum>(i), true);
              }
            int i = 0;
            ros::Duration delay(0.1);
            while(i<10)
              {
                for(int j =0;j<4;j++)
                  {
                    initial_pressure[j] += robot_state_handle.contact_pressure_[j];
                  }
                delay.sleep();
                i++;
              }
            for(int i = 0;i<4;i++)
              {
                initial_pressure[i] = initial_pressure[i]/10;
                std::cout<<"Initial contact pressure for leg "<<i<<" is : "<<initial_pressure[i]<<std::endl;
              }


        commands_buffer.writeFromNonRT(std::vector<double>(n_joints, 0.0));
        //log_data_srv_ = node_handle.advertiseService("/capture_log_data", &RosMpcController::logDataCapture, this);
        base_command_sub_ = node_handle.subscribe<free_gait_msgs::RobotState>("/desired_robot_state", 1, &RosMpcController::baseCommandCallback, this);
        //! WSHY: having problem with update foot contact in gazebo_state_hardware_interface, so
        //! use subscribe, for real hardware interface, there should no problem fro this.
        string contact_topic;
        if(real_robot)
          contact_topic = "/foot_contacts";
        else
          contact_topic = "/bumper_sensor_filter_node/foot_contacts";
        contact_sub_ = node_handle.subscribe<sim_assiants::FootContacts>(contact_topic, 1, &RosMpcController::footContactsCallback, this);
        //! WSHY: Logged data publisher
    //    joint_command_pub_ = node_handle.advertise<sensor_msgs::JointState>("/balance_controller/joint_command", 1);
    //    base_command_pub_ = node_handle.advertise<nav_msgs::Odometry>("/log/base_command", log_length_);
    //    base_actual_pub_ = node_handle.advertise<nav_msgs::Odometry>("/log/base_actual", log_length_);
        leg_state_pub_ = node_handle.advertise<std_msgs::Int8MultiArray>("/log/leg_state", log_length_);
        joint_command_pub_ = node_handle.advertise<sensor_msgs::JointState>("/log/joint_command", log_length_);
        joint_actual_pub_ = node_handle.advertise<sensor_msgs::JointState>("/log/joint_state", log_length_);
        contact_desired_pub_ = node_handle.advertise<sim_assiants::FootContacts>("/log/desired_foot_contact", log_length_);
        leg_phase_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("/log/leg_phase", log_length_);
        desired_robot_state_pub_ = node_handle.advertise<free_gait_msgs::RobotState>("/log/desired_robot_state", log_length_);
        actual_robot_state_pub_ = node_handle.advertise<free_gait_msgs::RobotState>("/log/actual_robot_state", log_length_);
        vmc_info_pub_ = node_handle.advertise<geometry_msgs::WrenchStamped>("/log/vmc_force_torque", log_length_);
        desired_vmc_info_pub_ = node_handle.advertise<geometry_msgs::WrenchStamped>("/log/desired_vmc_force_torque", log_length_);
        motor_status_word_pub_ = node_handle.advertise<std_msgs::Int8MultiArray>("/log/status_word", log_length_);

        base_command_pub_ = node_handle.advertise<nav_msgs::Odometry>("/log/base_command", log_length_);
        base_actual_pub_ = node_handle.advertise<nav_msgs::Odometry>("/log/base_actual", log_length_);
        time_test = ros::Time::now();
        ROS_INFO("Balance Controller initialized");
        return true;
    };
    /**
     * @brief RosMpcController::update, controller update loop
     * @param time
     * @param period
     */
    void RosMpcController::update(const ros::Time& time, const ros::Duration& period){
//        std::cout << period.toSec() << endl;
        ros::Time tmp = ros::Time::now();
        //std::cout <<"Time:"<< (tmp-time_test).toSec()<<"*" << endl;
        time_test = tmp;

        sensor_msgs::JointState joint_command, joint_actual;
        joint_command.effort.resize(12);
        joint_command.position.resize(12);
        joint_command.name.resize(12);
        joint_actual.name.resize(12);
        joint_actual.position.resize(12);
        joint_actual.velocity.resize(12);
        joint_actual.effort.resize(12);

        //control_steps_mpc.resize(4);
        free_gait::JointPositions all_joint_positions;
        free_gait::JointVelocities all_joint_velocities;
        free_gait::JointEfforts all_joint_efforts;
        std_msgs::Int8MultiArray status_word;
        status_word.data.resize(12);
        for(unsigned int i=0; i<12; i++)
          {
            all_joint_positions(i) = robot_state_handle.getJointPositionRead()[i];
            all_joint_velocities(i) = robot_state_handle.getJointVelocityRead()[i];
            all_joint_efforts(i) = robot_state_handle.getJointEffortRead()[i];
            joint_actual.position[i] = all_joint_positions(i);
            joint_actual.velocity[i] = all_joint_velocities(i);
            joint_actual.effort[i] = all_joint_efforts(i);
            status_word.data[i] = robot_state_handle.motor_status_word_[i];
          }
        boost::recursive_mutex::scoped_lock lock(r_mutex_);
        std::vector<double> & commands = *commands_buffer.readFromRT();

        std_msgs::Int8MultiArray leg_state;
        std_msgs::Float64MultiArray leg_phase;
        leg_phase.data.resize(8);
        leg_state.data.resize(4);
        RotationQuaternion base_orinetation = RotationQuaternion(robot_state_handle.getOrientation()[0],
             robot_state_handle.getOrientation()[1],
             robot_state_handle.getOrientation()[2],
             robot_state_handle.getOrientation()[3]);

        if(!ignore_contact_sensor)
          contactStateMachine();
        int num_of_stance_legs = 0;
        for(unsigned int i = 0; i < 4; i++)
        {
            free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
            //! WSHY: set foot cartesian motion for single leg controller
            robot_state->setTargetFootPositionInBaseForLimb(Position(foot_positions.at(limb).vector()), limb);
            robot_state->setTargetFootVelocityInBaseForLimb(LinearVelocity(foot_velocities.at(limb).vector()), limb);
            single_leg_solver_->setvecQAct(all_joint_positions.vector().segment(3*i, 3), limb);
            single_leg_solver_->setvecQDotAct(all_joint_velocities.vector().segment(3*i, 3), limb);
            if(ignore_contact_sensor)
              {/*
                limbs_state.at(limb)->setState(StateSwitcher::States::SwingNormal);
                if(limbs_desired_state.at(limb)->getState() == StateSwitcher::States::StanceNormal && st_phase.at(limb)>0.1)*/
                  limbs_state.at(limb)->setState(limbs_desired_state.at(limb)->getState());
              }
            switch (limbs_state.at(limb)->getState()) {
                case StateSwitcher::States::SwingNormal:
                            {
                              delay_counts[i] = 0;
                              robot_state_handle.foot_contact_[i] = 0;
                              robot_state->setSupportLeg(limb, false);
                              //            surface_normals.at(limb) = base_orinetation.rotate(Vector(0,0,1));
                              robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
                              //            robot_state->setSurfaceNormal(limb, robot_state_->getSurfaceNormal(limb));
                              leg_state.data[i] = 0;
                              store_current_joint_state_flag_.at(limb) = false;
                              update_surface_normal_flag.at(limb) = false;
                              stored_current_foot_position_flag.at(limb) = false;
                              break;
                            }
            case StateSwitcher::States::StanceNormal:
              {
                num_of_stance_legs++;
                delay_counts[i]++;
  //              robot_state->setSupportLeg(limb, true);
                //! WSHY: delay for base velocity estimate
                leg_state.data[i] = 1;
  //              robot_state_handle.foot_contact_[i] = 0;
                if(ignore_contact_sensor)
                  {

                    if(st_phase.at(limb)>0.2)
                      {

                        robot_state_handle.foot_contact_[i] = 1;
                        leg_state.data[i] = 2;
                      }
                  } else {
                    robot_state->setSupportLeg(limb, true);
                    leg_state.data[i] = 2;
  //                  if(delay_counts[i]>30) //for trot
                    if(delay_counts[i]>5)
                      {

                        robot_state_handle.foot_contact_[i] = 1;

                      }
  //                  if(st_phase.at(limb)>0.8)
  //                    {

  //                      robot_state_handle.foot_contact_[i] = 0;
  //                    }

                  }


                //            if(!update_surface_normal_flag.at(limb))
                //              {
                //                update_surface_normal_flag.at(limb) = true;
                //                Eigen::Matrix3d jacobian = robot_state_->getTranslationJacobianFromBaseToFootInBaseFrame(limb);
                //                free_gait::JointEffortsLeg joint_effort = free_gait::JointEffortsLeg(all_joint_efforts.vector().segment(i*3, 3));
                //                free_gait::Force contact_force = free_gait::Force(jacobian * joint_effort.toImplementation());
                //                surface_normals.at(limb) = Vector(contact_force.normalize());
                //                if(limb == free_gait::LimbEnum::LF_LEG || limb == free_gait::LimbEnum::RH_LEG)
                //                  surface_normals.at(limb) = Vector(-contact_force.normalize());
                //              }
                robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
                //            robot_state->setSurfaceNormal(limb, robot_state_->getSurfaceNormal(limb));
  //              leg_state.data[i] = 1;
                store_current_joint_state_flag_.at(limb) = false;
  //              ROS_INFO("Leg '%d' is in StanceNormal mode", i);
                break;
              }
            case StateSwitcher::States::SwingEarlyTouchDown:
              {
                robot_state_handle.foot_contact_[i] = 2;
  //              delay_counts[i]++;
  //              if(delay_counts[i]==10)
  //                {
  //                  delay_counts[i] = 0;
  //                  limbs_state.at(limb)->setState(StateSwitcher::States::StanceNormal);
  //                }
  //              robot_state->setSupportLeg(limb, true);
                if(!stored_current_foot_position_flag.at(limb))
                  {
                    stored_current_foot_position_flag.at(limb) = true;
                    stored_foot_positions.at(limb) = foot_positions.at(limb);
                  }else{
                    robot_state_->setTargetFootPositionInBaseForLimb(Position(stored_foot_positions.at(limb)),limb);
                    robot_state->setSupportLeg(limb, false);
                  }
  //              robot_state->setSupportLeg(limb, true);
                //              if(!update_surface_normal_flag.at(limb))
                //                {
                //                  update_surface_normal_flag.at(limb) = true;
                //                  Eigen::Matrix3d jacobian = robot_state_->getTranslationJacobianFromBaseToFootInBaseFrame(limb);
                //                  free_gait::JointEffortsLeg joint_effort = free_gait::JointEffortsLeg(all_joint_efforts.vector().segment(i*3, 3));
                //                  free_gait::Force contact_force = free_gait::Force(jacobian * joint_effort.toImplementation());
                //                  surface_normals.at(limb) = Vector(contact_force.normalize());
                //                  if(limb == free_gait::LimbEnum::LF_LEG || limb == free_gait::LimbEnum::RH_LEG)
                //                    surface_normals.at(limb) = Vector(-contact_force.normalize());
                //                }
                robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
                //              robot_state->setSurfaceNormal(limb, robot_state_->getSurfaceNormal(limb));
                //! WSHY: keep end effort position when early touch down
  //              ROS_WARN("Leg '%d' is in SwingEarlyTouchDown mode", i);
                leg_state.data[i] = 1;
                break;
              }
            case StateSwitcher::States::SwingBumpedIntoObstacle:
              {
                leg_state.data[i] = 4;
                robot_state_handle.foot_contact_[i] = 4;
                robot_state->setSupportLeg(limb, false);
                robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
                LinearVelocity desired_velocity_in_base = base_orinetation.inverseRotate(base_desired_linear_velocity);

  //              LinearVelocity current_velocity_in_base = base_orinetation.inverseRotate(LinearVelocity(robot_state_handle.getLinearVelocity()[0],
  //                                                                    robot_state_handle.getLinearVelocity()[1],
  //                                                                    robot_state_handle.getLinearVelocity()[2]));
                //! WSHY: move back(-x) and upward(+z)
                foot_positions.at(limb)(0) -= 0.005;
                foot_positions.at(limb)(2) += 0.02;
  //              foot_positions.at(limb).x() -= desired_velocity_in_base.x()*period.toSec();
  //              foot_positions.at(limb).y() -= desired_velocity_in_base.y()*period.toSec();
                //std::cout<<"desired velocity in base is : "<<desired_velocity_in_base<<std::endl;
                robot_state->setTargetFootPositionInBaseForLimb(Position(foot_positions.at(limb).vector()), limb);

                break;
              }
            case StateSwitcher::States::SwingLatelyTouchDown:
              {
                LinearVelocity current_velocity_in_base = base_orinetation.inverseRotate(LinearVelocity(robot_state_handle.getLinearVelocity()[0],
                                                                      robot_state_handle.getLinearVelocity()[1],
                                                                      robot_state_handle.getLinearVelocity()[2]));
                LinearVelocity desired_velocity_in_base = base_orinetation.inverseRotate(base_desired_linear_velocity);
                robot_state_handle.foot_contact_[i] = 3;
                /****************
              * TODO(Shunyao) : Directly move down?
              ****************/

                if(!stored_current_foot_position_flag.at(limb))
                  {
                    stored_current_foot_position_flag.at(limb) = true;
                    stored_foot_positions.at(limb) = foot_positions.at(limb);
                  }else{
                    stored_foot_positions.at(limb).z() -= 0.005;
                    stored_foot_positions.at(limb).x() += desired_velocity_in_base.x()*period.toSec();
                    stored_foot_positions.at(limb).y() += desired_velocity_in_base.y()*period.toSec();
                    if(stored_foot_positions.at(limb).z()<-0.6)
                      stored_foot_positions.at(limb).z() = -0.6;
                    robot_state_->setTargetFootPositionInBaseForLimb(Position(stored_foot_positions.at(limb)),limb);
                    robot_state->setSupportLeg(limb, false);
                    foot_positions.at(limb) = stored_foot_positions.at(limb);
                  }

                robot_state->setSupportLeg(limb, false);
                //            surface_normals.at(limb) = base_orinetation.rotate(Vector(0,0,1));
                robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
                //! WSHY: directly move down to ground
  //              foot_positions.at(limb).z() -= 0.005;
  //              foot_positions.at(limb).x() += desired_velocity_in_base.x()*period.toSec();
  //              foot_positions.at(limb).y() += desired_velocity_in_base.y()*period.toSec();
  //              std::cout<<"position diff X "<<current_velocity_in_base.x()*period.toSec()<<std::endl;

  //              robot_state->setTargetFootPositionInBaseForLimb(Position(foot_positions.at(limb).vector()), limb);
                //            robot_state->setSurfaceNormal(limb, robot_state_->getSurfaceNormal(limb));
                //! WSHY: keep end effort position when early touch down
                if(!store_current_joint_state_flag_.at(limb)){
                    store_current_joint_state_flag_.at(limb) = true;
                    //                stored_limb_joint_position_[0] = robot_state_handle.getJointPositionRead()[3*i];
                    //                stored_limb_joint_position_[1] = robot_state_handle.getJointPositionRead()[3*i + 1];
                    //                stored_limb_joint_position_[2] = robot_state_handle.getJointPositionRead()[3*i + 2];
                    stored_limb_joint_position_.vector().segment(3*i,3) = all_joint_positions.vector().segment(3*i,3);
                  } else {
                    commands[3*i] = stored_limb_joint_position_(3*i);
                    commands[3*i + 1] = stored_limb_joint_position_(3*i+1);
                    commands[3*i + 2] = stored_limb_joint_position_(3*i+2);
                  }
  //              ROS_WARN("Leg '%d' is in SwingLatelyTouchDown mode", i);
                leg_state.data[i] = 3;
                break;
              }
            case StateSwitcher::States::StanceLostContact:
              {
                robot_state_handle.foot_contact_[i] = 0;
                robot_state->setSupportLeg(limb, false);
                //! WSHY: directly move down to ground
                foot_positions.at(limb)(2) -= 0.01;
                robot_state->setTargetFootPositionInBaseForLimb(Position(foot_positions.at(limb).vector()), limb);

                //            surface_normals.at(limb) = base_orinetation.rotate(Vector(0,0,1));
                robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
                //            robot_state->setSurfaceNormal(limb, robot_state_->getSurfaceNormal(limb));
                //! WSHY: keep end effort position when early touch down
                if(!store_current_joint_state_flag_.at(limb)){
                    store_current_joint_state_flag_.at(limb) = true;
                    //                stored_limb_joint_position_[0] = robot_state_handle.getJointPositionRead()[3*i];
                    //                stored_limb_joint_position_[1] = robot_state_handle.getJointPositionRead()[3*i + 1];
                    //                stored_limb_joint_position_[2] = robot_state_handle.getJointPositionRead()[3*i + 2];
                    stored_limb_joint_position_.vector().segment(3*i,3) = all_joint_positions.vector().segment(3*i,3);
                  } else {
                    commands[3*i] = stored_limb_joint_position_(3*i);
                    commands[3*i + 1] = stored_limb_joint_position_(3*i+1);
                    commands[3*i + 2] = stored_limb_joint_position_(3*i+2);
                  }
                ROS_WARN("Leg '%d' is Lost Contact!!!", i);
                leg_state.data[i] = -2;
                break;
              }
            case StateSwitcher::States::Init:
              {
                robot_state->setSupportLeg(limb, true);
                //            surface_normals.at(limb) = base_orinetation.rotate(Vector(0,0,1));
                robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
                //            robot_state->setSurfaceNormal(limb, Vector(0, 0, 1));
                ROS_WARN("Leg '%d' is Init", i);
                break;
              }
            case StateSwitcher::States::SwingLateLiftOff:
              {
                robot_state_handle.foot_contact_[i] = 0;
                leg_state.data[i] = -1;
                robot_state->setSupportLeg(limb, false);
                robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
  //              ROS_WARN("Leg '%d' is in SwingLateLiftOff mode", i);
                break;
              }
            default:
              ROS_WARN("Unspecificed Limb State");

            }
  //        std::cout<<"surface normal for leg "<<i<<"("<<surface_normals.at(limb)<<")"<<std::endl;

  //        robot_state->setSupportLeg(limb, true);
  //       RotationQuaternion base_orinetation = RotationQuaternion(robot_state_handle.getOrientation()[0],
  //            robot_state_handle.getOrientation()[1],
  //            robot_state_handle.getOrientation()[2],
  //            robot_state_handle.getOrientation()[3]);
  //        robot_state->setSurfaceNormal(limb, Vector(0,0,1));
            robot_state->setSurfaceNormal(limb, base_orinetation.rotate(Vector(0,0,1)));

         }

        lock.unlock();
        //! WSHY: set the desired state
        robot_state->setPositionWorldToBaseInWorldFrame(base_desired_position);
        robot_state->setOrientationBaseToWorld(base_desired_rotation);
        robot_state->setLinearVelocityBaseInWorldFrame(base_desired_linear_velocity);
        robot_state->setAngularVelocityBaseInBaseFrame(base_desired_angular_velocity);



        //! WSHY: update current base state from robot state handle
        robot_state->setCurrentLimbJoints(all_joint_positions);
        robot_state->setCurrentLimbJointVelocities(all_joint_velocities);

    //    ROS_ERROR("State Estimate Position in X : %f",robot_state_handle.getPosition()[0]);
        Pose current_base_pose = Pose(Position(robot_state_handle.getPosition()[0],
                                      robot_state_handle.getPosition()[1],
                                      robot_state_handle.getPosition()[2]),
                             RotationQuaternion(robot_state_handle.getOrientation()[0],
                                                robot_state_handle.getOrientation()[1],
                                                robot_state_handle.getOrientation()[2],
                                                robot_state_handle.getOrientation()[3]));

        robot_state->setPoseBaseToWorld(current_base_pose);
        robot_state->setBaseStateFromFeedback(LinearVelocity(robot_state_handle.getLinearVelocity()[0],
                                                              robot_state_handle.getLinearVelocity()[1],
                                                              robot_state_handle.getLinearVelocity()[2]),
                                               LocalAngularVelocity(robot_state_handle.getAngularVelocity()[0],
                                                                    robot_state_handle.getAngularVelocity()[1],
                                                                    robot_state_handle.getAngularVelocity()[2]));
        /****************
    * TODO(Shunyao) : set support leg and surface normal, update from robot state handle
    ****************/

        //! WSHY: compute joint torque
        //std::cout <<"size:" << control_steps_mpc.size() << std::endl;
//        if(control_steps_mpc.empty()){
//            bool keep_flag = false;
//            if(!collections_4_mpc())
//              {
//                ROS_ERROR("MPC compute failed");
//                keep_flag = true;
//              }
//            vector<double> mpc_ans =  mpc_solver->ComputeContactForces(com_position, com_velocity, com_roll_pitch_yaw, com_angular_velocity,
//                                             foot_contact_states,foot_positions_body_frame ,foot_friction_coeffs,
//                                             desired_com_position, desired_com_velocity, desired_com_roll_pitch_yaw,desired_com_angular_velocity);
//            for (int i = 0; i < 2; i++) {
//                vector<double> impc(mpc_ans.begin() + 12*i, mpc_ans.begin()+12*i+12);
//                control_steps_mpc.push(impc);
//            }
//        }
//        vector<double> mpc_use = control_steps_mpc.front();
        Position footpos = robot_state->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::LF_LEG);
        //std::cout << "in mpc: "<<footpos.x() << " " << footpos.y() << " " << footpos.z()<< std::endl;
        bool keep_flag = false;
        int mit_use = 1;
        bool _pybullet = true;
        CtrlEnum ctrlx = CtrlEnum::MIT;
        if(!collections_4_mpc(mit_use))
          {
            ROS_ERROR("MPC compute failed");
            keep_flag = true;
          }
        //golaoxu: test input com_xyz as a {0.0}
        //TODO:*********************************
        vector<double> mpc_ans =  mpc_solver->ComputeContactForces(com_position, com_velocity, com_roll_pitch_yaw, com_angular_velocity,
                                         foot_contact_states,foot_positions_body_frame ,foot_friction_coeffs,
                                         desired_com_position, desired_com_velocity, desired_com_roll_pitch_yaw,desired_com_angular_velocity);
//        vector<double> mpc_ans =  mpc_solver->ComputeContactForces({0.0}, com_velocity, com_roll_pitch_yaw, com_angular_velocity,
//                                         foot_contact_states,foot_positions_body_frame ,foot_friction_coeffs,
//                                         desired_com_position, desired_com_velocity, desired_com_roll_pitch_yaw,desired_com_angular_velocity);
        vector<double> mpc_use(mpc_ans.begin(), mpc_ans.begin()+12);
//        std::cout.precision(2);
//        std::cout << endl;
        for (int i = 1; i < mpc_use.size(); i= i+3) {
            mpc_use[i] = mpc_use[i];
            //std::cout << mpc_use[i] << "~~";
        }

//        for(int i = 0; i < 3; i++){
//            double tmp = mpc_use[i+6];
//            mpc_use[i+6] = mpc_use[i+9];
//            mpc_use[i+9] = tmp;
//        }
//        std::cout << "LF_force: ";
//        for (int i = 0; i < 3; i++){
//            //mpc_use[i] = -mpc_use[i];
//            std::cout << mpc_use[i] << " - ";
//        }
//        std::cout << std::endl;
        computeJointTorques(mpc_use,ctrlx);
        for(int i = 0; i<4; i++)
          {
            /****************
             * TODO(Shunyao) :  decide support leg to apply joint torque
             ****************/
    //        if(robot_state_)
    //        double joint_torque_command = robot_state->getAllJointEfforts()(i);

            free_gait::JointEffortsLeg joint_torque_limb = robot_state->getJointEffortsForLimb(static_cast<free_gait::LimbEnum>(i));
//            for (int x = 0; x < 3; ++x) {
//               std::cout << joint_torque_limb(i) << " !! ";
//            }
    //        for(int k=0;k<3;k++)
    //          joint_command.effort[i*3 + k] = joint_torque_limb(k);
    //        jointTorquesLimit(joint_torque_limb, 60.0);
            int start_index = i*3;
            for(int j =0;j<3;j++)
              {
                double joint_torque_command = joint_torque_limb(j);
                int index = start_index + j;

                ROS_DEBUG("Torque computed of joint %d is : %f\n",index,joint_torque_command);

                robot_state_handle.getJointEffortWrite()[index] = joint_torque_command;
                robot_state_handle.mode_of_joint_[i] = 4;
                //        if(joint_torque_command>300)
                //          joint_torque_command = 300;
                //        if(joint_torque_command<-300)
                //          joint_torque_command = -300;

                //        if(!keep_flag)
                //          {
                //            joints[i].setCommand(joint_torque_command);
                //          }
                joint_command.name[index] = joint_names[index];
    //            if(num_of_stance_legs>1)
                joint_command.effort[index] = joint_torque_command;
                joint_command.position[index] = commands[index];
                joint_actual.name[index] = joint_names[index];
//                std::cout <<"-" << joint_torque_command;

    //            joint_actual.position[index] = all_joint_positions(index);
    //            joint_actual.velocity[index] = all_joint_velocities(index);
    //            joint_actual.effort[index] = all_joint_efforts(index);
              }
          }
        //ssswww
        //ssswww
        //ssswww
        ros::Duration real_time_period = ros::Duration(period.toSec());
        //golaoxu : for the test
        //TODO :change this real_time_period  in gazobo interface.cpp !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //real_time_period = 0.01;
        //std::cout <<"real time period "<< real_time_period << endl;
        free_gait::Force gravity_in_base = base_orinetation.rotate(free_gait::Force(0,0,-9.8));
        if(!robot_state->isSupportLeg(free_gait::LimbEnum::LF_LEG))
          {
    //        ROS_INFO("LF_LEG is NOT Contacted");
            //! WSHY: compute gravity compensation
            free_gait::JointPositionsLeg joint_position_leg = free_gait::JointPositionsLeg(all_joint_positions.vector().segment(0,3));
            free_gait::JointEffortsLeg gravity_compensation_torque = robot_state->getGravityCompensationForLimb(free_gait::LimbEnum::LF_LEG,
                                                                                    joint_position_leg,
                                                                                    gravity_in_base);
    //        single_leg_solver_->setvecQAct(joint_position_leg.vector(),free_gait::LimbEnum::LF_LEG);
            single_leg_solver_->update(time, real_time_period, free_gait::LimbEnum::LF_LEG,
                                       real_robot, foot_accelerations.at(free_gait::LimbEnum::LF_LEG).vector());

            for(int i = 0;i<3;i++)
              {
                double effort_command = computeTorqueFromPositionCommand(commands[i], i, real_time_period);
                if(is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) || is_footstep_.at(free_gait::LimbEnum::LF_LEG))
                  effort_command = single_leg_solver_->getVecTauAct()[i];
                else if(!is_legmode_.at(free_gait::LimbEnum::LF_LEG))
                  {
                    //! WSHY: joint control mode, use Profile Position mode;
                    robot_state_handle.mode_of_joint_[i] = 1;
                    if(!real_robot)
                      effort_command += gravity_compensation_torque(i);
                    position_joints[i].setCommand(commands[i]);
                    //ROS_INFO("Joint control");
                  }
                else
                  effort_command = gravity_compensation_torque(i);

                joints[i].setCommand(effort_command);
                //! WSHY: data logging
                joint_command.effort[i] = effort_command;
    //            joint_actual.position[i] = single_leg_solver_->getQAcutal().row(1)[i];
    //            joint_actual.velocity[i] = single_leg_solver_->getQDotAcutal().row(1)[i];
    //            joint_actual.effort[i] = single_leg_solver_->getQDDotAcutal().row(1)[i];
              }
            //        ROS_INFO("LF_LEG is NOT Contacted");
          }else{ // for stance leg
            for(unsigned int i=0;i<3;i++)
              {
                joints[i].setCommand(joint_command.effort[i]);
                //ROS_INFO("Torque : %lf", joint_command.effort[i]);
              }
          }
        if(!robot_state->isSupportLeg(free_gait::LimbEnum::RF_LEG))
          {

            free_gait::JointPositionsLeg joint_position_leg = free_gait::JointPositionsLeg(all_joint_positions.vector().segment(3,3));
            free_gait::JointEffortsLeg gravity_compensation_torque = robot_state->getGravityCompensationForLimb(free_gait::LimbEnum::RF_LEG,
                                                                                    joint_position_leg,
                                                                                    gravity_in_base);

    //        single_leg_solver_->setvecQAct(joint_position_leg.vector(),free_gait::LimbEnum::RF_LEG);
    //        single_leg_solver_->update(time, real_time_period, free_gait::LimbEnum::RF_LEG);
            single_leg_solver_->update(time, real_time_period, free_gait::LimbEnum::RF_LEG,
                                       real_robot, foot_accelerations.at(free_gait::LimbEnum::RF_LEG).vector());
            for(int i = 0;i<3;i++)
              {
                double effort_command = computeTorqueFromPositionCommand(commands[i+3], i+3, real_time_period);
                if(is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) || is_footstep_.at(free_gait::LimbEnum::RF_LEG))
                  effort_command = single_leg_solver_->getVecTauAct()[i];
                else if(!is_legmode_.at(free_gait::LimbEnum::RF_LEG))
                  {
                    //! WSHY: joint control mode, use Profile Position mode;
                    robot_state_handle.mode_of_joint_[i+3] = 1;
                    if(!real_robot)
                      effort_command += gravity_compensation_torque(i);
                    position_joints[i+3].setCommand(commands[i+3]);
                    //ROS_INFO("Joint control");
                  }
                else
                  effort_command = gravity_compensation_torque(i);
                joints[i+3].setCommand(effort_command);
                joint_command.effort[i+3] = effort_command;

    //            joint_actual.position[i+3] = single_leg_solver_->getQAcutal().row(1)[i];
    //            joint_actual.velocity[i+3] = single_leg_solver_->getQDotAcutal().row(1)[i];
    //            joint_actual.effort[i+3] = single_leg_solver_->getTauAcutal().row(0)[i]; //single_leg_solver_->getQDDotAcutal().row(1)[i];
              }
    //                ROS_INFO("RF_LEG is NOT Contacted");
          }else{
            for(unsigned int i=0;i<3;i++)
              {
                joints[i+3].setCommand(joint_command.effort[i+3]);
              }
          }
        if(!robot_state->isSupportLeg(free_gait::LimbEnum::RH_LEG))
          {

            free_gait::JointPositionsLeg joint_position_leg = free_gait::JointPositionsLeg(all_joint_positions.vector().segment(6,3));
            free_gait::JointEffortsLeg gravity_compensation_torque = robot_state->getGravityCompensationForLimb(free_gait::LimbEnum::RH_LEG,
                                                                                    joint_position_leg,
                                                                                    gravity_in_base);

    //        single_leg_solver_->setvecQAct(joint_position_leg.vector(),free_gait::LimbEnum::RH_LEG);
    //        single_leg_solver_->update(time, real_time_period, free_gait::LimbEnum::RH_LEG);
            single_leg_solver_->update(time, real_time_period, free_gait::LimbEnum::RH_LEG,
                                       real_robot, foot_accelerations.at(free_gait::LimbEnum::RH_LEG).vector());
            for(int i = 0;i<3;i++)
              {
                double effort_command =computeTorqueFromPositionCommand(commands[i+6], i+6, real_time_period);
                if(is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) || is_footstep_.at(free_gait::LimbEnum::RH_LEG))
                  effort_command = single_leg_solver_->getVecTauAct()[i];
                else if(!is_legmode_.at(free_gait::LimbEnum::RH_LEG))
                  {
                    //! WSHY: joint control mode, use Profile Position mode;
                    robot_state_handle.mode_of_joint_[i+6] = 1;
                    if(!real_robot)
                      effort_command += gravity_compensation_torque(i);
                    position_joints[i+6].setCommand(commands[i+6]);
                    //ROS_INFO("Joint control");
                  }
                else
                  effort_command = gravity_compensation_torque(i);
                joints[i+6].setCommand(effort_command);
                joint_command.effort[i+6] = effort_command;

    //            joint_actual.position[i+6] = single_leg_solver_->getQAcutal().row(1)[i];
    //            joint_actual.velocity[i+6] = single_leg_solver_->getQDotAcutal().row(1)[i];
    //            joint_actual.effort[i+6] = single_leg_solver_->getTauAcutal().row(0)[i]; //single_leg_solver_->getQDDotAcutal().row(1)[i];
              }
    //                ROS_INFO("RH_LEG is NOT Contacted");
          }else{
            for(unsigned int i=0;i<3;i++)
              {
                joints[i+6].setCommand(joint_command.effort[i+6]);
              }
          }
        if(!robot_state->isSupportLeg(free_gait::LimbEnum::LH_LEG))
          {
            free_gait::JointPositionsLeg joint_position_leg = free_gait::JointPositionsLeg(all_joint_positions.vector().segment(9,3));
            free_gait::JointEffortsLeg gravity_compensation_torque = robot_state->getGravityCompensationForLimb(free_gait::LimbEnum::LH_LEG,
                                                                                    joint_position_leg,
                                                                                    gravity_in_base);
    //        single_leg_solver_->setvecQAct(joint_position_leg.vector(),free_gait::LimbEnum::LH_LEG);
    //        single_leg_solver_->update(time, real_time_period, free_gait::LimbEnum::LH_LEG);
            single_leg_solver_->update(time, real_time_period, free_gait::LimbEnum::LH_LEG,
                                       real_robot, foot_accelerations.at(free_gait::LimbEnum::LH_LEG).vector());

            for(int i = 0;i<3;i++)
              {
                double effort_command =computeTorqueFromPositionCommand(commands[i+9], i+9, real_time_period);
                if(is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) || is_footstep_.at(free_gait::LimbEnum::LH_LEG))
                  effort_command = single_leg_solver_->getVecTauAct()[i];
                else if(!is_legmode_.at(free_gait::LimbEnum::LH_LEG))
                  {
                    //! WSHY: joint control mode, use Profile Position mode;
                    robot_state_handle.mode_of_joint_[i+9] = 1;
                    if(!real_robot)
                      effort_command += gravity_compensation_torque(i);
                    position_joints[i+9].setCommand(commands[i+9]);
                    //ROS_INFO("Joint control");
                  }
                else
                  effort_command = gravity_compensation_torque(i);
                joints[i+9].setCommand(effort_command);
                joint_command.effort[i+9] = effort_command;

    //            joint_actual.position[i+9] = single_leg_solver_->getQAcutal().row(1)[i];
    //            joint_actual.velocity[i+9] = single_leg_solver_->getQDotAcutal().row(1)[i];
    //            joint_actual.effort[i+9] = single_leg_solver_->getTauAcutal().row(0)[i]; //single_leg_solver_->getQDDotAcutal().row(1)[i];
              }
    //                ROS_INFO("LH_LEG is NOT Contacted");
          }else{
            for(unsigned int i=0;i<3;i++)
              {
                joints[i+9].setCommand(joint_command.effort[i+9]);
              }
          }
    //    lock.unlock();
    }
    double RosMpcController::computeTorqueFromPositionCommand(double command, int i, const ros::Duration& period)
    {
        double command_position = command;

        double error; //, vel_error;
        double commanded_effort;

        double current_position = joints[i].getPosition();

        // Make sure joint is within limits if applicable
        enforceJointLimits(command_position, i);

        // Compute position error
        if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE)
        {
         angles::shortest_angular_distance_with_limits(
            current_position,
            command_position,
            joint_urdfs_[i]->limits->lower,
            joint_urdfs_[i]->limits->upper,
            error);
        }
        else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS)
        {
          error = angles::shortest_angular_distance(current_position, command_position);
        }
        else //prismatic
        {
          error = command_position - current_position;
        }

        // Set the PID error and compute the PID command with nonuniform
        // time step size.
        commanded_effort = pid_controllers_[i].computeCommand(error, period);
        return commanded_effort;
    }
    void RosMpcController::baseCommandCallback(const free_gait_msgs::RobotStateConstPtr& robot_state_msg){
        base_desired_position = Position(robot_state_msg->base_pose.pose.pose.position.x,
                                          robot_state_msg->base_pose.pose.pose.position.y,
                                          robot_state_msg->base_pose.pose.pose.position.z);
        base_desired_rotation = RotationQuaternion(robot_state_msg->base_pose.pose.pose.orientation.w,
                                                              robot_state_msg->base_pose.pose.pose.orientation.x,
                                                              robot_state_msg->base_pose.pose.pose.orientation.y,
                                                              robot_state_msg->base_pose.pose.pose.orientation.z);
        base_desired_linear_velocity = LinearVelocity(robot_state_msg->base_pose.twist.twist.linear.x,
                                                                     robot_state_msg->base_pose.twist.twist.linear.y,
                                                                     robot_state_msg->base_pose.twist.twist.linear.z);
        base_desired_angular_velocity = LocalAngularVelocity(robot_state_msg->base_pose.twist.twist.angular.x,
                                                                                  robot_state_msg->base_pose.twist.twist.angular.y,
                                                                                  robot_state_msg->base_pose.twist.twist.angular.z);

    //    Pose current_base_pose = Pose(Position(robot_state_handle.getPosition()[0],
    //                                  robot_state_handle.getPosition()[1],
    //                                  robot_state_handle.getPosition()[2]),
    //                         RotationQuaternion(robot_state_handle.getOrientation()[0],
    //                                            robot_state_handle.getOrientation()[1],
    //                                            robot_state_handle.getOrientation()[2],
    //                                            robot_state_handle.getOrientation()[3]));
    //    ROS_INFO("Desired base pose: Position: ");
    //    std::cout<<base_desired_position<<std::endl;
    //    kindr::EulerAnglesZyxPD rotation_desired(base_desired_rotation);
    //    ROS_INFO("Desired base pose: Rotation: ");
    //    std::cout<<rotation_desired<<std::endl;
    //    ROS_INFO("Current base pose: Position: ");
    //    std::cout<<current_base_pose.getPosition()<<std::endl;
    //    kindr::EulerAnglesZyxPD rotation_current(current_base_pose.getRotation());
    //    ROS_INFO("Current base pose: Rotation: ");
    //    std::cout<<rotation_current<<std::endl;
    //    ROS_INFO("Orienitaion Error is : ");
    //    std::cout<<base_desired_rotation.boxMinus(current_base_pose.getRotation())<<std::endl;

    //    free_gait::JointPositions all_joint_positions;
        std::vector<double> joint_commands;
        joint_commands.resize(12);
        for(unsigned int i = 0;i<3;i++)
        {
            /****************
            * TODO(Shunyao) : only for the non-support leg to follow the joint position and
            *velocity command
            ****************/
          joint_commands[i] = robot_state_msg->lf_leg_joints.position[i];
          joint_commands[i+3] = robot_state_msg->rf_leg_joints.position[i];
          joint_commands[i+6] = robot_state_msg->rh_leg_joints.position[i];
          joint_commands[i+9] = robot_state_msg->lh_leg_joints.position[i];
        }
        commands_buffer.writeFromNonRT(joint_commands);

    //    LimbVector foot_positions, foot_velocities, foot_accelerations;
        //! WSHY: update cartesian foot motion command
        Position foot_position, foot_velocity, foot_acceleration;

        kindr_ros::convertFromRosGeometryMsg(robot_state_msg->lf_target.target_position[0].point,
            foot_position);
        foot_positions[free_gait::LimbEnum::LF_LEG] = Vector(foot_position.toImplementation());
        kindr_ros::convertFromRosGeometryMsg(robot_state_msg->lf_target.target_velocity[0].vector,
            foot_velocity);
        foot_velocities[free_gait::LimbEnum::LF_LEG] = Vector(foot_velocity.toImplementation());
        kindr_ros::convertFromRosGeometryMsg(robot_state_msg->lf_target.target_acceleration[0].vector,
            foot_acceleration);
        foot_accelerations[free_gait::LimbEnum::LF_LEG] = Vector(foot_acceleration.toImplementation());

        kindr_ros::convertFromRosGeometryMsg(robot_state_msg->rf_target.target_position[0].point,
            foot_position);
        foot_positions[free_gait::LimbEnum::RF_LEG] = Vector(foot_position.toImplementation());
        kindr_ros::convertFromRosGeometryMsg(robot_state_msg->rf_target.target_velocity[0].vector,
            foot_velocity);
        foot_velocities[free_gait::LimbEnum::RF_LEG] = Vector(foot_velocity.toImplementation());
        kindr_ros::convertFromRosGeometryMsg(robot_state_msg->rf_target.target_acceleration[0].vector,
            foot_acceleration);
        foot_accelerations[free_gait::LimbEnum::RF_LEG] = Vector(foot_acceleration.toImplementation());

        kindr_ros::convertFromRosGeometryMsg(robot_state_msg->rh_target.target_position[0].point,
            foot_position);
        foot_positions[free_gait::LimbEnum::RH_LEG] = Vector(foot_position.toImplementation());
        kindr_ros::convertFromRosGeometryMsg(robot_state_msg->rh_target.target_velocity[0].vector,
            foot_velocity);
        foot_velocities[free_gait::LimbEnum::RH_LEG] = Vector(foot_velocity.toImplementation());
        kindr_ros::convertFromRosGeometryMsg(robot_state_msg->rh_target.target_acceleration[0].vector,
            foot_acceleration);
        foot_accelerations[free_gait::LimbEnum::RH_LEG] = Vector(foot_acceleration.toImplementation());

        kindr_ros::convertFromRosGeometryMsg(robot_state_msg->lh_target.target_position[0].point,
            foot_position);
        foot_positions[free_gait::LimbEnum::LH_LEG] = Vector(foot_position.toImplementation());
        kindr_ros::convertFromRosGeometryMsg(robot_state_msg->lh_target.target_velocity[0].vector,
            foot_velocity);
        foot_velocities[free_gait::LimbEnum::LH_LEG] = Vector(foot_velocity.toImplementation());
        kindr_ros::convertFromRosGeometryMsg(robot_state_msg->lh_target.target_acceleration[0].vector,
            foot_acceleration);
        foot_accelerations[free_gait::LimbEnum::LH_LEG] = Vector(foot_acceleration.toImplementation());


        //std::cout << "LF_POSITION:"<<foot_positions[free_gait::LimbEnum::LF_LEG] <<endl;

    //    command_foot_buffer.writeFromNonRT(foot_positions);
    //    command_foot_vel_buffer.writeFromNonRT(foot_velocities);

        robot_state_->setPositionWorldToBaseInWorldFrame(base_desired_position);
        robot_state_->setOrientationBaseToWorld(RotationQuaternion(base_desired_rotation));
        robot_state_->setLinearVelocityBaseInWorldFrame(base_desired_linear_velocity);
        robot_state_->setAngularVelocityBaseInBaseFrame(base_desired_angular_velocity);

        if(robot_state_msg->lf_leg_mode.name == "joint")
          {
            is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = false;
            is_footstep_.at(free_gait::LimbEnum::LF_LEG) = false;
            is_legmode_.at(free_gait::LimbEnum::LF_LEG) = false;
          }
        if(robot_state_msg->lf_leg_mode.name == "leg_mode")
          {
            is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = false;
            is_footstep_.at(free_gait::LimbEnum::LF_LEG) = false;
            is_legmode_.at(free_gait::LimbEnum::LF_LEG) = true;
          }
        if(robot_state_msg->lf_leg_mode.name == "cartesian")
          {
            is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = true;
            is_footstep_.at(free_gait::LimbEnum::LF_LEG) = false;
            is_legmode_.at(free_gait::LimbEnum::LF_LEG) = false;
          }
        if(robot_state_msg->lf_leg_mode.name == "footstep")
          {
            is_footstep_.at(free_gait::LimbEnum::LF_LEG) = true;
            is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = false;
            is_legmode_.at(free_gait::LimbEnum::LF_LEG) = false;
          }

        if(robot_state_msg->rf_leg_mode.name == "joint")
          {
            is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = false;
            is_footstep_.at(free_gait::LimbEnum::RF_LEG) = false;
            is_legmode_.at(free_gait::LimbEnum::RF_LEG) = false;
          }
        if(robot_state_msg->rf_leg_mode.name == "leg_mode")
          {
            is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = false;
            is_footstep_.at(free_gait::LimbEnum::RF_LEG) = false;
            is_legmode_.at(free_gait::LimbEnum::RF_LEG) = true;
          }
        if(robot_state_msg->rf_leg_mode.name == "cartesian")
          {
            is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = true;
            is_footstep_.at(free_gait::LimbEnum::RF_LEG) = false;
            is_legmode_.at(free_gait::LimbEnum::RF_LEG) = false;
          }
        if(robot_state_msg->rf_leg_mode.name == "footstep")
          {
            is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = false;
            is_footstep_.at(free_gait::LimbEnum::RF_LEG) = true;
            is_legmode_.at(free_gait::LimbEnum::RF_LEG) = false;
          }

        if(robot_state_msg->rh_leg_mode.name == "joint")
          {
            is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = false;
            is_footstep_.at(free_gait::LimbEnum::RH_LEG) = false;
            is_legmode_.at(free_gait::LimbEnum::RH_LEG) = false;
          }
        if(robot_state_msg->rh_leg_mode.name == "leg_mode")
          {
            is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = false;
            is_footstep_.at(free_gait::LimbEnum::RH_LEG) = false;
            is_legmode_.at(free_gait::LimbEnum::RH_LEG) = true;
          }
        if(robot_state_msg->rh_leg_mode.name == "cartesian")
          {
            is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = true;
            is_footstep_.at(free_gait::LimbEnum::RH_LEG) = false;
            is_legmode_.at(free_gait::LimbEnum::RH_LEG) = false;
          }
        if(robot_state_msg->rh_leg_mode.name == "footstep")
          {
            is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = false;
            is_footstep_.at(free_gait::LimbEnum::RH_LEG) = true;
            is_legmode_.at(free_gait::LimbEnum::RH_LEG) = false;
          }

        if(robot_state_msg->lh_leg_mode.name == "joint")
          {
            is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = false;
            is_footstep_.at(free_gait::LimbEnum::LH_LEG) = false;
            is_legmode_.at(free_gait::LimbEnum::LH_LEG) = false;
          }
        if(robot_state_msg->lh_leg_mode.name == "leg_mode")
          {
            is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = false;
            is_footstep_.at(free_gait::LimbEnum::LH_LEG) = false;
            is_legmode_.at(free_gait::LimbEnum::LH_LEG) = true;
          }
        if(robot_state_msg->lh_leg_mode.name == "cartesian")
          {
            is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = true;
            is_footstep_.at(free_gait::LimbEnum::LH_LEG) = false;
            is_legmode_.at(free_gait::LimbEnum::LH_LEG) = false;
          }
        if(robot_state_msg->lh_leg_mode.name == "footstep")
          {
            is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = false;
            is_footstep_.at(free_gait::LimbEnum::LH_LEG) = true;
            is_legmode_.at(free_gait::LimbEnum::LH_LEG) = false;
          }

        if(robot_state_msg->lf_leg_mode.support_leg){
            robot_state_->setSupportLeg(free_gait::LimbEnum::LF_LEG, true);
            robot_state_->setSurfaceNormal(free_gait::LimbEnum::LF_LEG,
                                           Vector(robot_state_msg->lf_leg_mode.surface_normal.vector.x,
                                                  robot_state_msg->lf_leg_mode.surface_normal.vector.y,
                                                  robot_state_msg->lf_leg_mode.surface_normal.vector.z));

            limbs_desired_state.at(free_gait::LimbEnum::LF_LEG)->setState(StateSwitcher::States::StanceNormal);
            if(st_flag.at(free_gait::LimbEnum::LF_LEG)){
                st_flag.at(free_gait::LimbEnum::LF_LEG) = false;
                t_st0.at(free_gait::LimbEnum::LF_LEG)= ros::Time::now();
              }
            sw_flag.at(free_gait::LimbEnum::LF_LEG) = false;
            st_phase.at(free_gait::LimbEnum::LF_LEG) = robot_state_msg->lf_leg_mode.phase;
            sw_phase.at(free_gait::LimbEnum::LF_LEG) = 0;
          } else {
    //        ROS_WARN("NO Contact !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

            robot_state_->setSupportLeg(free_gait::LimbEnum::LF_LEG, false);
            robot_state_->setSurfaceNormal(free_gait::LimbEnum::LF_LEG,Vector(0,0,1));
            limbs_desired_state.at(free_gait::LimbEnum::LF_LEG)->setState(StateSwitcher::States::SwingNormal);
            if(!sw_flag.at(free_gait::LimbEnum::LF_LEG)){
              t_sw0.at(free_gait::LimbEnum::LF_LEG) = ros::Time::now();
              sw_flag.at(free_gait::LimbEnum::LF_LEG) = true;
              }
            st_flag.at(free_gait::LimbEnum::LF_LEG) = true;
            sw_phase.at(free_gait::LimbEnum::LF_LEG) = robot_state_msg->lf_leg_mode.phase;
            st_phase.at(free_gait::LimbEnum::LF_LEG) = 0;

          };
        if(robot_state_msg->rf_leg_mode.support_leg){
            robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(1), true);
            robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(1),
                                           Vector(robot_state_msg->rf_leg_mode.surface_normal.vector.x,
                                                  robot_state_msg->rf_leg_mode.surface_normal.vector.y,
                                                  robot_state_msg->rf_leg_mode.surface_normal.vector.z));
            limbs_desired_state.at(free_gait::LimbEnum::RF_LEG)->setState(StateSwitcher::States::StanceNormal);
            if(st_flag.at(free_gait::LimbEnum::RF_LEG)){
                st_flag.at(free_gait::LimbEnum::RF_LEG) = false;
                t_st0.at(free_gait::LimbEnum::RF_LEG)= ros::Time::now();
              }
            sw_flag.at(free_gait::LimbEnum::RF_LEG) = false;
            st_phase.at(free_gait::LimbEnum::RF_LEG) = robot_state_msg->rf_leg_mode.phase;
            sw_phase.at(free_gait::LimbEnum::RF_LEG) = 0;
          } else {
    //        ROS_WARN("NO Contact !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(1), false);
            robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(1),Vector(0,0,1));
            limbs_desired_state.at(free_gait::LimbEnum::RF_LEG)->setState(StateSwitcher::States::SwingNormal);
            if(!sw_flag.at(free_gait::LimbEnum::RF_LEG)){
              t_sw0.at(free_gait::LimbEnum::RF_LEG) = ros::Time::now();
              sw_flag.at(free_gait::LimbEnum::RF_LEG) = true;
              }
            st_flag.at(free_gait::LimbEnum::RF_LEG) = true;
            sw_phase.at(free_gait::LimbEnum::RF_LEG) = robot_state_msg->rf_leg_mode.phase;
            st_phase.at(free_gait::LimbEnum::RF_LEG) = 0;

          };
        if(robot_state_msg->rh_leg_mode.support_leg){
            robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(2), true);
            robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(2),
                                           Vector(robot_state_msg->rh_leg_mode.surface_normal.vector.x,
                                                  robot_state_msg->rh_leg_mode.surface_normal.vector.y,
                                                  robot_state_msg->rh_leg_mode.surface_normal.vector.z));
            limbs_desired_state.at(free_gait::LimbEnum::RH_LEG)->setState(StateSwitcher::States::StanceNormal);
            if(st_flag.at(free_gait::LimbEnum::RH_LEG)){
                st_flag.at(free_gait::LimbEnum::RH_LEG) = false;
                t_st0.at(free_gait::LimbEnum::RH_LEG)= ros::Time::now();
              }
            sw_flag.at(free_gait::LimbEnum::RH_LEG) = false;
            st_phase.at(free_gait::LimbEnum::RH_LEG) = robot_state_msg->rh_leg_mode.phase;
            sw_phase.at(free_gait::LimbEnum::RH_LEG) = 0;
          } else {
    //        ROS_WARN("NO Contact !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(2), false);
            robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(2),Vector(0,0,1));
            limbs_desired_state.at(free_gait::LimbEnum::RH_LEG)->setState(StateSwitcher::States::SwingNormal);
            if(!sw_flag.at(free_gait::LimbEnum::RH_LEG)){
              t_sw0.at(free_gait::LimbEnum::RH_LEG) = ros::Time::now();
              sw_flag.at(free_gait::LimbEnum::RH_LEG) = true;
              }
            st_flag.at(free_gait::LimbEnum::RH_LEG) = true;
            sw_phase.at(free_gait::LimbEnum::RH_LEG) = robot_state_msg->rh_leg_mode.phase;
            st_phase.at(free_gait::LimbEnum::RH_LEG) = 0;

          };
        if(robot_state_msg->lh_leg_mode.support_leg){
            robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(3), true);
            robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(3),
                                           Vector(robot_state_msg->lh_leg_mode.surface_normal.vector.x,
                                                  robot_state_msg->lh_leg_mode.surface_normal.vector.y,
                                                  robot_state_msg->lh_leg_mode.surface_normal.vector.z));
            limbs_desired_state.at(free_gait::LimbEnum::LH_LEG)->setState(StateSwitcher::States::StanceNormal);
            if(st_flag.at(free_gait::LimbEnum::LH_LEG)){
                st_flag.at(free_gait::LimbEnum::LH_LEG) = false;
                t_st0.at(free_gait::LimbEnum::LH_LEG)= ros::Time::now();
              }
            sw_flag.at(free_gait::LimbEnum::LH_LEG) = false;
            st_phase.at(free_gait::LimbEnum::LH_LEG) = robot_state_msg->lh_leg_mode.phase;
            sw_phase.at(free_gait::LimbEnum::LH_LEG) = 0;
          } else {
    //        ROS_WARN("NO Contact !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(3), false);
            robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(3),Vector(0,0,1));
            limbs_desired_state.at(free_gait::LimbEnum::LH_LEG)->setState(StateSwitcher::States::SwingNormal);
            if(!sw_flag.at(free_gait::LimbEnum::LH_LEG)){
              t_sw0.at(free_gait::LimbEnum::LH_LEG) = ros::Time::now();
              sw_flag.at(free_gait::LimbEnum::LH_LEG) = true;
              }
            st_flag.at(free_gait::LimbEnum::LH_LEG) = true;
            sw_phase.at(free_gait::LimbEnum::LH_LEG) = robot_state_msg->lh_leg_mode.phase;
            st_phase.at(free_gait::LimbEnum::LH_LEG) = 0;
          };
      }

    void RosMpcController::contactStateMachine()
      {

        for(unsigned int i = 0;i<4;i++)
          {
            free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);

            if(real_robot && !ignore_contact_sensor)
              {
                real_contact_force_.at(limb).z() = robot_state_handle.contact_pressure_[i];
                if((robot_state_handle.contact_pressure_[i] - initial_pressure[i]) > contact_pressure_bias)
                  {
    //                robot_state_handle.foot_contact_[i] = 1;
                    real_contact_.at(limb) = true;
                  }else {
    //                robot_state_handle.foot_contact_[i] = 0;
                    real_contact_.at(limb) = false;
                  }
              }

    //        if(real_robot)
    //          {
    ////            if(robot_state_handle.contact_pressure_[i] > 1)
    ////              real_contact_.at(limb) = true;
    ////            else
    ////              real_contact_.at(limb) = false;
    //            real_contact_.at(limb) = robot_state_handle.foot_contact_[i];

    //          }

            if(limbs_desired_state.at(limb)->getState() == StateSwitcher::States::SwingNormal)
              {
                if(ignore_contact_sensor)
                  continue;
    //            if(!is_footstep_.at(limb) && !is_cartisian_motion_.at(limb))
    //              continue;
                if(real_contact_.at(limb) && (limbs_state.at(limb)->getState() == StateSwitcher::States::StanceNormal
                                              || limbs_state.at(limb)->getState() == StateSwitcher::States::SwingLateLiftOff))
                  {
                    limbs_state.at(limb)->setState(StateSwitcher::States::SwingLateLiftOff);
                    continue;
                  }
                if(!real_contact_.at(limb) && (limbs_state.at(limb)->getState() == StateSwitcher::States::SwingLateLiftOff
                                            || limbs_state.at(limb)->getState() == StateSwitcher::States::SwingBumpedIntoObstacle
                                            || limbs_state.at(limb)->getState() == StateSwitcher::States::SwingNormal
                                            || limbs_state.at(limb)->getState() == StateSwitcher::States::StanceNormal))
                  {
                    limbs_state.at(limb)->setState(StateSwitcher::States::SwingNormal);
                    continue;
                  }
                if(sw_phase.at(limb)>0.5 && real_contact_.at(limb) && limbs_state.at(limb)->getState() == StateSwitcher::States::SwingNormal)
                  {
                    limbs_state.at(limb)->setState(StateSwitcher::States::SwingEarlyTouchDown);
                    continue;
                  }
    //            if(real_contact_.at(limb) && limbs_state.at(limb)->getLastState() == StateSwitcher::States::SwingNormal)
    //              {
    //                limbs_state.at(limb)->setState(StateSwitcher::States::SwingBumpedIntoObstacle);
    //                continue;
    //              }
    //            if(real_contact_.at(limb) && limbs_state.at(limb)->getLastState() == StateSwitcher::States::SwingBumpedIntoObstacle)
    //              {
    //                limbs_state.at(limb)->setState(StateSwitcher::States::SwingNormal);
    //                continue;
    //              }

              }

            if(limbs_desired_state.at(limb)->getState() == StateSwitcher::States::StanceNormal)
              {
                if(ignore_contact_sensor)
                  {
                    limbs_state.at(limb)->setState(StateSwitcher::States::StanceNormal);
                    continue;
                  }

    //            if(!is_footstep_.at(limb) && !is_cartisian_motion_.at(limb))
    //              {
    //                if(real_contact_.at(limb))
    //                  limbs_state.at(limb)->setState(StateSwitcher::States::StanceNormal);
    //                else
    //                  limbs_state.at(limb)->setState(StateSwitcher::States::SwingNormal);
    //                continue;
    //              }

                if(limbs_state.at(limb)->getState()==StateSwitcher::States::SwingEarlyTouchDown)
                  {
                    limbs_state.at(limb)->setState(StateSwitcher::States::StanceNormal);
                    continue;
                  }
                if(real_contact_.at(limb)
                   && (limbs_state.at(limb)->getState()==StateSwitcher::States::SwingNormal
                   || limbs_state.at(limb)->getState()==StateSwitcher::States::SwingLatelyTouchDown))
                  {
                    limbs_state.at(limb)->setState(StateSwitcher::States::StanceNormal);
                    continue;
                  }
                if(!real_contact_.at(limb) && (limbs_state.at(limb)->getState()==StateSwitcher::States::SwingNormal))
                  {
                    limbs_state.at(limb)->setState(StateSwitcher::States::SwingLatelyTouchDown);
                    continue;
                  }
                if(real_contact_.at(limb) && limbs_state.at(limb)->getState()==StateSwitcher::States::StanceNormal)
                {
                    limbs_state.at(limb)->setState(StateSwitcher::States::StanceNormal);
                    continue;
                  }

    //            if(!real_contact_.at(limb) && limbs_state.at(limb)->getLastState()==StateSwitcher::States::StanceNormal)
    //            {
    //                limbs_state.at(limb)->setState(StateSwitcher::States::StanceLostContact);
    //               continue;
    //            }


          }


          }
      }
    void RosMpcController::footContactsCallback(const sim_assiants::FootContactsConstPtr& foot_contacts)
      {
        /****************
    * TODO(Shunyao) : change contact state for the early or late contact
    ****************/
    //    unsigned int i = 0;
        for(unsigned int i = 0;i<foot_contacts->foot_contacts.size();i++)
          {
            auto contact = foot_contacts->foot_contacts[i];
            free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
    //        ROS_INFO("swing time for leg %d is : %f", i, t_sw0.at(limb).toSec());
            real_contact_.at(limb) = contact.is_contact;
            real_contact_force_.at(limb).z() = contact.contact_force.wrench.force.z;
          }
      }

    void RosMpcController::starting(const ros::Time& time)
      {
    //    for(int i = 0;i<4;i++)
    //      robot_state_handle.foot_contact_[i] = 1;
    //    base_command_pose_.clear();
    //    base_actual_pose_.clear();
        leg_states_.clear();
        joint_actual_.clear();
        joint_command_.clear();
        foot_desired_contact_.clear();
        leg_phases_.clear();
        desired_robot_state_.clear();
        actual_robot_state_.clear();
        vitual_force_torque_.clear();
        desired_vitual_force_torque_.clear();
        log_time_.clear();
        motor_status_word_.clear();
        base_actual_pose_.clear();
        base_command_pose_.clear();
        for(int i = 0;i<joints.size();i++)
          {
            joints[i].setCommand(robot_state_handle.getJointEffortRead()[i]);
          }
        for(int i =0;i<4;i++)
          {
            limbs_state.at(static_cast<free_gait::LimbEnum>(i))->setState(StateSwitcher::States::StanceNormal);
            robot_state->setSupportLeg(static_cast<free_gait::LimbEnum>(i), true);
          }
      };

    void RosMpcController::stopping(const ros::Time& time)
      {
        for(int i = 0;i<joints.size();i++)
          {
            joints[i].setCommand(0);
          }
      };

    void RosMpcController::enforceJointLimits(double &command, unsigned int index)
        {
          // Check that this joint has applicable limits
          if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE || joint_urdfs_[index]->type == urdf::Joint::PRISMATIC)
          {
            if( command > joint_urdfs_[index]->limits->upper ) // above upper limnit
            {
              command = joint_urdfs_[index]->limits->upper;
            }
            else if( command < joint_urdfs_[index]->limits->lower ) // below lower limit
            {
              command = joint_urdfs_[index]->limits->lower;
            }
          }
        }

    bool RosMpcController::jointTorquesLimit(free_gait::JointEffortsLeg& joint_torque, double max_torque)
      {
    //    int max_index;
        double max_value=0;
        for(int i=0;i<3;i++)
          {
            if(fabs(joint_torque(i))>max_value)
              {
                max_value = fabs(joint_torque(i));
    //            max_index = i;
              }
          }
        if(max_value<max_torque)
          max_torque = max_value;
        double ratio = max_torque/max_value;
        for(int i=0;i<3;i++)
          {
            joint_torque(i) = joint_torque(i)*ratio;
          }
        return true;
    }
    bool RosMpcController::QuaternionToEuler_desired(){
        vector<double> euler;
        euler.resize(3);
        const double Epsilon = 0.0009765625f;
            const double Threshold = 0.5f - Epsilon;

            double TEST = desired_quaternion[3] * desired_quaternion[1] - desired_quaternion[0] * desired_quaternion[2];

            if (TEST < -Threshold || TEST > Threshold) // 奇异姿态,俯仰角为±90°
            {
                int sign = 1;
                if(TEST <= -1e-7){
                    sign = -1;
                }
                euler[2] = -2 * sign * (double)atan2(desired_quaternion[0], desired_quaternion[3]); // yaw

                euler[1] = sign * (EIGEN_PI / 2.0); // pitch

                euler[0] = 0; // roll

            }
            else
            {
                euler[0] = atan2(2 * (desired_quaternion[1]*desired_quaternion[2] + desired_quaternion[3]*desired_quaternion[0]), desired_quaternion[3]*desired_quaternion[3] - desired_quaternion[0]*desired_quaternion[0] - desired_quaternion[1]*desired_quaternion[1] + desired_quaternion[2]*desired_quaternion[2]);
                euler[1] = asin(-2 * (desired_quaternion[0]*desired_quaternion[2] - desired_quaternion[3]*desired_quaternion[1]));
                euler[2] = atan2(2 * (desired_quaternion[0]*desired_quaternion[1] + desired_quaternion[3]*desired_quaternion[2]), desired_quaternion[3]*desired_quaternion[3] + desired_quaternion[0]*desired_quaternion[0] - desired_quaternion[1]*desired_quaternion[1] - desired_quaternion[2]*desired_quaternion[2]);
            }
            desired_com_roll_pitch_yaw.assign(euler.begin(), euler.end());
            return true;
    }
    bool RosMpcController::QuaternionToEuler(){
        vector<double> euler;
        euler.resize(3);
        const double Epsilon = 0.0009765625f;
            const double Threshold = 0.5f - Epsilon;

            double TEST = quaternion[3] * quaternion[1] - quaternion[0] * quaternion[2];

            if (TEST < -Threshold || TEST > Threshold) // 奇异姿态,俯仰角为±90°
            {
                int sign = 1;
                if(TEST <= -1e-7){
                    sign = -1;
                }
                //euler[2] = -2 * sign * (double)atan2(desired_quaternion[0], desired_quaternion[3]); // yaw
                euler[2] = -2 * sign * (double)atan2(quaternion[0], quaternion[3]); // yaw

                euler[1] = sign * (EIGEN_PI / 2.0); // pitch

                euler[0] = 0; // roll

            }
            else
            {
                euler[0] = atan2(2 * (quaternion[1]*quaternion[2] + quaternion[3]*quaternion[0]), quaternion[3]*quaternion[3] - quaternion[0]*quaternion[0] - quaternion[1]*quaternion[1] + quaternion[2]*quaternion[2]);
                euler[1] = asin(-2 * (quaternion[0]*quaternion[2] - quaternion[3]*quaternion[1]));
                euler[2] = atan2(2 * (quaternion[0]*quaternion[1] + quaternion[3]*quaternion[2]), quaternion[3]*quaternion[3] + quaternion[0]*quaternion[0] - quaternion[1]*quaternion[1] - quaternion[2]*quaternion[2]);
            }
            com_roll_pitch_yaw.assign(euler.begin(), euler.end());
            return true;
    }

    //Golaoxu : following params has some problems ;
    bool RosMpcController::collections_4_mpc(bool ispybulletways){
        //std::cout << "Test mpc:" << endl;
        //1.com_position
        Position _com_position = robot_state_->getPositionWorldToBaseInWorldFrame();
        com_position.resize(3);
        //Golaoxu: in MIT's ways, the x,y and z is been set to 0.
        //TODO:
        //test:*********************************************
        com_position[0] = _com_position.x();
        com_position[1] = _com_position.y();
        com_position[0] = 0.0;
        com_position[1] = 0.0;
        com_position[2] = _com_position.z();
        //cout << "zzzzz" << com_position[2]<<endl;

        //2.com_velocity
        LinearVelocity _com_velocity = robot_state_->getLinearVelocityBaseInWorldFrame();
        com_velocity.resize(3);
        com_velocity[0] = _com_velocity.x();
        com_velocity[1] = _com_velocity.y();
        com_velocity[2] = _com_velocity.z();
        //std::cout << " com vel x:" <<com_velocity[0] << std::endl;
        //3.rpy
        com_roll_pitch_yaw.resize(3);
        RotationQuaternion _rotation_orien= robot_state_->getOrientationBaseToWorld();
        quaternion.resize(4);
        quaternion[0] = _rotation_orien.x();
        quaternion[1] = _rotation_orien.y();
        quaternion[2] = _rotation_orien.z();
        quaternion[3] = _rotation_orien.w();
        QuaternionToEuler();
        //com_roll_pitch_yaw[2] = 0;
        //Golaoxu: yaw is set to 0;
        //don't consider the yaw angel
        //test:********************************************
        //com_roll_pitch_yaw[2] = 0;
        //std::cout << "yaw" << com_roll_pitch_yaw[2] << std::endl;
        //4.ang_vel
        LocalAngularVelocity _com_angular_velocity = robot_state_->getAngularVelocityBaseInBaseFrame();
        com_angular_velocity.resize(3);
        com_angular_velocity[0] = _com_angular_velocity.x();
        com_angular_velocity[1] = _com_angular_velocity.y();
        com_angular_velocity[2] = _com_angular_velocity.z();
        //5.contact_states
        foot_contact_states.resize(4);
        for (const auto& leg : limbs_){
            int _n;
            switch (leg) {
                case free_gait::LimbEnum::LF_LEG:
                    _n = 0;
                    break;
                case free_gait::LimbEnum::RF_LEG:
                    _n = 1;
                    break;
                case free_gait::LimbEnum::RH_LEG:
                    _n = 2;
                    break;
                case free_gait::LimbEnum::LH_LEG:
                    _n = 3;
                    break;
                default:
                    ROS_ERROR("no leg be chosen");
            }
            if(robot_state_->isSupportLeg(leg)){
                foot_contact_states[_n] = 1;
            }
            else{
                foot_contact_states[_n] = 0;
            }
        }
        //6.foot_postion
        foot_positions_body_frame.resize(12);
        for(const auto& leg: limbs_){
            int _n;
            switch (leg) {
                case free_gait::LimbEnum::LF_LEG:
                    _n = 0;
                    break;
                case free_gait::LimbEnum::RF_LEG:
                    _n = 1;
                    break;
                case free_gait::LimbEnum::RH_LEG:
                    _n = 2;
                    break;
                case free_gait::LimbEnum::LH_LEG:
                    _n = 3;
                    break;
                default:
                    ROS_ERROR("no leg be chosen");
            }
            //Golaoxu : the foot posotion perhaps is wrong
            //TODO:
            // 1. transfer the foot position in world frame to MPC directly.
            // 2. in MPC_class ,change the function
            // 1,2 is Wrong:
            Position footpos = robot_state->getPositionBaseToFootInBaseFrame(leg);
            //Position footpos_in_world = robot_state->getPositionWorldToFootInWorldFrame(leg);

            foot_positions_body_frame[_n * 3]  = footpos.x();
            foot_positions_body_frame[_n * 3 + 1]  = footpos.y();
            foot_positions_body_frame[_n * 3 + 2]  = footpos.z();
        }
//        std::cout << "LF: " << foot_positions_body_frame[0] <<" " <<
//                     foot_positions_body_frame[1]<<" "<<
//                     foot_positions_body_frame[2]<< endl;
//        std::cout << "RF: " << foot_positions_body_frame[3] <<" " <<
//                     foot_positions_body_frame[4]<<" "<<
//                     foot_positions_body_frame[5]<< endl;
//        std::cout << "RH: " << foot_positions_body_frame[6] <<" " <<
//                     foot_positions_body_frame[7]<<" "<<
//                     foot_positions_body_frame[8]<< endl;
//        std::cout << "LH: " << foot_positions_body_frame[9] <<" " <<
//                     foot_positions_body_frame[10]<<" "<<
//                     foot_positions_body_frame[11]<< endl;
        //7.frition_coeffs
        foot_friction_coeffs = {0.45, 0.45, 0.45, 0.45};
        //8.desire_pos
        Position _desired_com_position = robot_state_->getTargetPositionWorldToBaseInWorldFrame();
        desired_com_position.resize(3);

//        std::cout << "Target CoM position: "<< robot_state_->getTargetPositionWorldToBaseInWorldFrame().x()<<" "<<
//                  robot_state_->getTargetPositionWorldToBaseInWorldFrame().y()<<" "<<
//                  robot_state_->getTargetPositionWorldToBaseInWorldFrame().z()<< std::endl;

//        std::cout << "Real CoM position: "<< robot_state_->getPositionWorldToBaseInWorldFrame().x()<< " "<<
//                  robot_state_->getPositionWorldToBaseInWorldFrame().y()<<" "<<
//                  robot_state_->getPositionWorldToBaseInWorldFrame().z()<< std::endl;
        //Golaoxu: in MIT ,the x,y is set to 0, and z is 0.42 which  is a desired CoM height.
        //TODO:
        desired_com_position[0] = _desired_com_position.x();
        desired_com_position[1] = _desired_com_position.y();
        //test:*********************************************
        desired_com_position[0] = 0.0;
        desired_com_position[1] = 0.0;
        //cout << "Dedired_com_z: " <<_desired_com_position.z()  << endl;
        desired_com_position[2] = _desired_com_position.z();
        desired_com_position[2] = 0.40;
        //cout << "des zzzzzzzzz" << desired_com_position[2] << endl;
        //9.desire_vel
        LinearVelocity _desired_com_velocity = robot_state_->getTargetLinearVelocityBaseInWorldFrame();
        desired_com_velocity.resize(3);
        //Golaoxu : in MIT , z vel is set to 0
        desired_com_velocity[0] = _desired_com_velocity.x();
        desired_com_velocity[1] = _desired_com_velocity.y();
        desired_com_velocity[2] = _desired_com_velocity.z();
        desired_com_velocity[2] = 0.0;
        //std::cout << "desired vel x" <<desired_com_velocity[0] << std::endl;
        //test:*********************************************
        //desired_com_velocity[2] = 0.0;
        //10.desire_rpy
        //Golaoxu : in MIT. rpy in this is set to 0 all; and yaw is compute by integrate by angular velocity
        //TODO:
        _rotation_orien = robot_state_->getTargetOrientationBaseToWorld();
        desired_quaternion.resize(4);
        desired_quaternion[0] = _rotation_orien.x();
        desired_quaternion[1] = _rotation_orien.y();
        desired_quaternion[2] = _rotation_orien.z();
        desired_quaternion[3] = _rotation_orien.w();
        desired_com_roll_pitch_yaw.resize(3);
        QuaternionToEuler_desired();
        desired_com_roll_pitch_yaw[0] = 0.0;
        desired_com_roll_pitch_yaw[1] = 0.0;
        desired_com_roll_pitch_yaw[2] = 0.0;
        //std::cout<<"desired yaw:" << desired_com_roll_pitch_yaw[2] << std::endl;

        //11.desire_ang_vel
        LocalAngularVelocity _desired_com_angular_velocity = robot_state_->getTargetAngularVelocityBaseInBaseFrame();
        desired_com_angular_velocity.resize(3);
        desired_com_angular_velocity[0] = _desired_com_angular_velocity.x();
        desired_com_angular_velocity[1] = _desired_com_angular_velocity.y();
        //Golaoxu : in MIT. angular vel in x y this is set to 0 all;
        //TODO:
        desired_com_angular_velocity[0] = 0.0;
        desired_com_angular_velocity[1] = 0.0;
        desired_com_angular_velocity[2] = _desired_com_angular_velocity.z();
        //std::cout << " dedired com vel ang z:" << desired_com_angular_velocity[2] << endl;
        //golaoxu: set the yaw vel as 0, which means that the robot cant rotate through command from user.
        //desired_com_angular_velocity[2] = 0.0;

        return true;
    }
    bool RosMpcController::collections_4_mpc(){
        //std::cout << "Test mpc:" << endl;
        //1.com_position
        Position _com_position = robot_state_->getPositionWorldToBaseInWorldFrame();
        com_position.resize(3);
        //Golaoxu: in MIT's ways, the x,y and z is been set to 0.
        //TODO:
        //test:*********************************************
        com_position[0] = _com_position.x();
        com_position[1] = _com_position.y();
        com_position[0] = 0.0;
        com_position[1] = 0.0;
        com_position[2] = _com_position.z();
        //2.com_velocity
        LinearVelocity _com_velocity = robot_state_->getLinearVelocityBaseInWorldFrame();
        com_velocity.resize(3);
        com_velocity[0] = _com_velocity.x();
        com_velocity[1] = _com_velocity.y();
        com_velocity[2] = _com_velocity.z();
        //std::cout << " com vel x:" <<com_velocity[0] << std::endl;
        //3.rpy
        com_roll_pitch_yaw.resize(3);
        RotationQuaternion _rotation_orien= robot_state_->getOrientationBaseToWorld();
        quaternion.resize(4);
        quaternion[0] = _rotation_orien.x();
        quaternion[1] = _rotation_orien.y();
        quaternion[2] = _rotation_orien.z();
        quaternion[3] = _rotation_orien.w();
        QuaternionToEuler();
        com_roll_pitch_yaw[1] = com_roll_pitch_yaw[1];
        //Golaoxu: yaw is set to 0;
        //don't consider the yaw angel
        //test:********************************************
        //com_roll_pitch_yaw[2] = 0;
        //std::cout << "yaw" << com_roll_pitch_yaw[2] << std::endl;
        //4.ang_vel
        LocalAngularVelocity _com_angular_velocity = robot_state_->getAngularVelocityBaseInBaseFrame();
        com_angular_velocity.resize(3);
        com_angular_velocity[0] = _com_angular_velocity.x();
        com_angular_velocity[1] = _com_angular_velocity.y();
        com_angular_velocity[2] = _com_angular_velocity.z();
        //5.contact_states
        foot_contact_states.resize(4);
        for (const auto& leg : limbs_){
            int _n;
            switch (leg) {
                case free_gait::LimbEnum::LF_LEG:
                    _n = 0;
                    break;
                case free_gait::LimbEnum::RF_LEG:
                    _n = 1;
                    break;
                case free_gait::LimbEnum::RH_LEG:
                    _n = 3;
                    break;
                case free_gait::LimbEnum::LH_LEG:
                    _n = 2;
                    break;
                default:
                    ROS_ERROR("no leg be chosen");
            }
            if(robot_state_->isSupportLeg(leg)){
                foot_contact_states[_n] = 1;
            }
            else{
                foot_contact_states[_n] = 0;
            }
        }
        //6.foot_postion
        foot_positions_body_frame.resize(12);
        for(const auto& leg: limbs_){
            int _n;
            switch (leg) {
                case free_gait::LimbEnum::LF_LEG:
                    _n = 0;
                    break;
                case free_gait::LimbEnum::RF_LEG:
                    _n = 1;
                    break;
                case free_gait::LimbEnum::RH_LEG:
                    _n = 3;
                    break;
                case free_gait::LimbEnum::LH_LEG:
                    _n = 2;
                    break;
                default:
                    ROS_ERROR("no leg be chosen");
            }
            //Golaoxu : the foot posotion perhaps is wrong
            //TODO:
            // 1. transfer the foot position in world frame to MPC directly.
            // 2. in MPC_class ,change the function
            // 1,2 is Wrong:
            Position footpos = robot_state->getPositionBaseToFootInBaseFrame(leg);
            //Position footpos_in_world = robot_state->getPositionWorldToFootInWorldFrame(leg);

            foot_positions_body_frame[_n * 3]  = footpos.x();
            foot_positions_body_frame[_n * 3 + 1]  = footpos.y();
            foot_positions_body_frame[_n * 3 + 2]  = footpos.z();
        }
//        std::cout << "LF: " << foot_positions_body_frame[0] <<" " <<
//                     foot_positions_body_frame[1]<<" "<<
//                     foot_positions_body_frame[2]<< endl;
//        std::cout << "RF: " << foot_positions_body_frame[3] <<" " <<
//                     foot_positions_body_frame[4]<<" "<<
//                     foot_positions_body_frame[5]<< endl;
//        std::cout << "RH: " << foot_positions_body_frame[6] <<" " <<
//                     foot_positions_body_frame[7]<<" "<<
//                     foot_positions_body_frame[8]<< endl;
//        std::cout << "LH: " << foot_positions_body_frame[9] <<" " <<
//                     foot_positions_body_frame[10]<<" "<<
//                     foot_positions_body_frame[11]<< endl;
        //7.frition_coeffs
        foot_friction_coeffs = {0.60, 0.60, 0.60, 0.60};
        //8.desire_pos
        Position _desired_com_position = robot_state_->getTargetPositionWorldToBaseInWorldFrame();
        desired_com_position.resize(3);

//        std::cout << "Target CoM position: "<< robot_state_->getTargetPositionWorldToBaseInWorldFrame().x()<<" "<<
//                  robot_state_->getTargetPositionWorldToBaseInWorldFrame().y()<<" "<<
//                  robot_state_->getTargetPositionWorldToBaseInWorldFrame().z()<< std::endl;

//        std::cout << "Real CoM position: "<< robot_state_->getPositionWorldToBaseInWorldFrame().x()<< " "<<
//                  robot_state_->getPositionWorldToBaseInWorldFrame().y()<<" "<<
//                  robot_state_->getPositionWorldToBaseInWorldFrame().z()<< std::endl;
        //Golaoxu: in MIT ,the x,y is set to 0, and z is 0.42 which  is a desired CoM height.
        //TODO:
        desired_com_position[0] = _desired_com_position.x();
        desired_com_position[1] = _desired_com_position.y();
        //test:*********************************************
        desired_com_position[0] = 0.0;
        desired_com_position[1] = 0.0;
        //cout << "Dedired_com_z: " <<_desired_com_position.z()  << endl;
        desired_com_position[2] = _desired_com_position.z();
        //9.desire_vel
        LinearVelocity _desired_com_velocity = robot_state_->getTargetLinearVelocityBaseInWorldFrame();
        desired_com_velocity.resize(3);
        //Golaoxu : in MIT , z vel is set to 0
        desired_com_velocity[0] = _desired_com_velocity.x();
        desired_com_velocity[1] = _desired_com_velocity.y();
        desired_com_velocity[2] = _desired_com_velocity.z();
        //std::cout << "desired vel x" <<desired_com_velocity[0] << std::endl;
        //test:*********************************************
        //desired_com_velocity[2] = 0.0;
        //10.desire_rpy
        //Golaoxu : in MIT. rpy in this is set to 0 all; and yaw is compute by integrate by angular velocity
        //TODO:
        _rotation_orien = robot_state_->getTargetOrientationBaseToWorld();
        desired_quaternion.resize(4);
        desired_quaternion[0] = _rotation_orien.x();
        desired_quaternion[1] = _rotation_orien.y();
        desired_quaternion[2] = _rotation_orien.z();
        desired_quaternion[3] = _rotation_orien.w();
        desired_com_roll_pitch_yaw.resize(3);
        QuaternionToEuler_desired();
        desired_com_roll_pitch_yaw[0] = 0.0;
        desired_com_roll_pitch_yaw[1] = 0.0;
        //desired_com_roll_pitch_yaw[2] = 0.0;
        //std::cout<<"desired yaw:" << desired_com_roll_pitch_yaw[2] << std::endl;

        //11.desire_ang_vel
        LocalAngularVelocity _desired_com_angular_velocity = robot_state_->getTargetAngularVelocityBaseInBaseFrame();
        desired_com_angular_velocity.resize(3);
        desired_com_angular_velocity[0] = _desired_com_angular_velocity.x();
        desired_com_angular_velocity[1] = _desired_com_angular_velocity.y();
        //Golaoxu : in MIT. angular vel in x y this is set to 0 all;
        //TODO:
        desired_com_angular_velocity[0] = 0.0;
        desired_com_angular_velocity[1] = 0.0;
        desired_com_angular_velocity[2] = _desired_com_angular_velocity.z();
        //std::cout << " dedired com vel ang z:" << desired_com_angular_velocity[2] << endl;
        //golaoxu: set the yaw vel as 0, which means that the robot cant rotate through command from user.
        //desired_com_angular_velocity[2] = 0.0;

        return true;
    };

    bool RosMpcController::collections_4_mpc(int ways_mit){
        if(!ways_mit){
            return false;
        }
        const RotationQuaternion& orientationControlToBase = robot_state_->getOrientationBaseToWorld().inverted();//torso_->getMeasuredState().getOrientationControlToBase();
        const RotationQuaternion& orientationWorldToBase = robot_state_->getOrientationBaseToWorld().inverted();//torso_->getMeasuredState().getOrientationWorldToBase();
        const RotationQuaternion& orientationWorldToControl = orientationWorldToBase*orientationControlToBase.inverted();//robot_state_->getTargetOrientationBaseToWorld();//torso_->getMeasuredState().getOrientationWorldToControl();

        //1.com_position
        Position _com_position = robot_state_->getPositionWorldToBaseInWorldFrame();
        com_position.resize(3);
        com_position[0] = _com_position.x();
        com_position[1] = _com_position.y();
        com_position[2] = _com_position.z();

        com_position[0] = 0.0;
        com_position[1] = 0.0;

        //2.com_velocity
        LinearVelocity _com_velocity = robot_state_->getLinearVelocityBaseInWorldFrame();
        com_velocity.resize(3);
        com_velocity[0] = _com_velocity.x();
        com_velocity[1] = _com_velocity.y();
        com_velocity[2] = _com_velocity.z();
        //3.rpy
        com_roll_pitch_yaw.resize(3);
        RotationQuaternion _rotation_orien= robot_state_->getOrientationBaseToWorld();
        quaternion.resize(4);
        quaternion[0] = _rotation_orien.x();
        quaternion[1] = _rotation_orien.y();
        quaternion[2] = _rotation_orien.z();
        quaternion[3] = _rotation_orien.w();

        QuaternionToEuler();
        //std::cout<<"yaw" << com_roll_pitch_yaw[2] << " ";
        //com_roll_pitch_yaw[2] = 0;

        //don't consider the yaw angel
        com_roll_pitch_yaw[2] = 0;
        //4.ang_vel


       // LocalAngularVelocity _com_angular_velocity =  robot_state_->getAngularVelocityBaseInBaseFrame();

        LocalAngularVelocity _com_angular_velocity =  orientationControlToBase.rotate(robot_state_->getAngularVelocityBaseInBaseFrame());
        com_angular_velocity.resize(3);
        com_angular_velocity[0] = _com_angular_velocity.x();
        com_angular_velocity[1] = _com_angular_velocity.y();
        com_angular_velocity[2] = _com_angular_velocity.z();
        //5.contact_states
        foot_contact_states.resize(4);
        for (const auto& leg : limbs_){
            int _n;
            switch (leg) {
                case free_gait::LimbEnum::LF_LEG:
                    _n = 0;
                    break;
                case free_gait::LimbEnum::RF_LEG:
                    _n = 1;
                    break;
                case free_gait::LimbEnum::RH_LEG:
                    _n = 2;
                    break;
                case free_gait::LimbEnum::LH_LEG:
                    _n = 3;
                    break;
                default:
                    ROS_ERROR("no leg be chosen");
            }
            if(robot_state_->isSupportLeg(leg)){
                foot_contact_states[_n] = 1;
            }
            else{
                foot_contact_states[_n] = 0;
            }
        }
        //6.foot_postion
        foot_positions_body_frame.resize(12);
        for(const auto& leg: limbs_){
            int _n;
            switch (leg) {
                case free_gait::LimbEnum::LF_LEG:
                    _n = 0;
                    break;
                case free_gait::LimbEnum::RF_LEG:
                    _n = 1;
                    break;
                case free_gait::LimbEnum::RH_LEG:
                    _n = 2;
                    break;
                case free_gait::LimbEnum::LH_LEG:
                    _n = 3;
                    break;
                default:
                    ROS_ERROR("no leg be chosen");
            }
            Position footpos = robot_state_->getPositionBaseToFootInBaseFrame(leg);
            foot_positions_body_frame[_n * 3]  = footpos.x();
            foot_positions_body_frame[_n * 3 + 1]  = footpos.y();
            foot_positions_body_frame[_n * 3 + 2]  = footpos.z();
        }
    //    std::cout << "Real foot position: " << foot_positions_body_frame[0] <<" " <<
    //                 foot_positions_body_frame[1]<<" "<<
    //                 foot_positions_body_frame[2]<< endl;
        //7.frition_coeffs
        foot_friction_coeffs = {0.6, 0.6, 0.6, 0.6};
        //8.desire_pos
        Position _desired_com_position = robot_state_->getTargetPositionWorldToBaseInWorldFrame();
        desired_com_position.resize(3);
        desired_com_position[0] = _desired_com_position.x();
        desired_com_position[1] = _desired_com_position.y();
        desired_com_position[2] = _desired_com_position.z();

        desired_com_position[0] = 0.0;
        desired_com_position[1] = 0.0;
        //0319
        desired_com_position[2] = 0.36;

        //9.desire_vel
        LinearVelocity _desired_com_velocity = robot_state_->getTargetLinearVelocityBaseInWorldFrame();
        desired_com_velocity.resize(3);
        desired_com_velocity[0] = _desired_com_velocity.x();
        desired_com_velocity[1] = _desired_com_velocity.y();
        desired_com_velocity[2] = _desired_com_velocity.z();
        desired_com_velocity[2] = 0.0;

        //0319
        desired_com_velocity[0] = 0;
        desired_com_velocity[1] = 0;
        desired_com_velocity[2] = 0;

        //10.desire_rpy
        _rotation_orien = robot_state_->getTargetOrientationBaseToWorld();
        desired_quaternion.resize(4);
        desired_quaternion[0] = _rotation_orien.x();
        desired_quaternion[1] = _rotation_orien.y();
        desired_quaternion[2] = _rotation_orien.z();
        desired_quaternion[3] = _rotation_orien.w();
        desired_com_roll_pitch_yaw.resize(3);
        QuaternionToEuler_desired();
    //    std::cout << "desired rpy " <<
    //                 desired_com_roll_pitch_yaw[2] << " "<< std::endl;
        desired_com_roll_pitch_yaw[0] = 0.0;
        desired_com_roll_pitch_yaw[1] = 0.0;
        desired_com_roll_pitch_yaw[2] = 0.0;

    //    std::cout << " rpy " <<
    //                 com_roll_pitch_yaw[2] << " "<< std::endl;

        //11.desire_ang_vel
        LocalAngularVelocity _desired_com_angular_velocity = robot_state_->getTargetAngularVelocityBaseInBaseFrame();
        desired_com_angular_velocity.resize(3);
        desired_com_angular_velocity[0] = _desired_com_angular_velocity.x();
        desired_com_angular_velocity[1] = _desired_com_angular_velocity.y();
        desired_com_angular_velocity[2] = _desired_com_angular_velocity.z();
        desired_com_angular_velocity[0] = 0.0;
        desired_com_angular_velocity[1] = 0.0;
        //0319
        //Golaoxu : this has great influence of mpc controller
        desired_com_angular_velocity[2] = 0.0;

        return  true;
    }

    bool RosMpcController::collections_4_mpc(CtrlEnum ctrl){
        if(ctrl == CtrlEnum::MIT){
            const RotationQuaternion& orientationControlToBase = robot_state_->getOrientationBaseToWorld().inverted();//torso_->getMeasuredState().getOrientationControlToBase();
            const RotationQuaternion& orientationWorldToBase = robot_state_->getOrientationBaseToWorld().inverted();//torso_->getMeasuredState().getOrientationWorldToBase();
            const RotationQuaternion& orientationWorldToControl = orientationWorldToBase*orientationControlToBase.inverted();//robot_state_->getTargetOrientationBaseToWorld();//torso_->getMeasuredState().getOrientationWorldToControl();

            //1.com_position
            Position _com_position = robot_state_->getPositionWorldToBaseInWorldFrame();
            com_position.resize(3);
            com_position[0] = _com_position.x();
            com_position[1] = _com_position.y();
            com_position[2] = _com_position.z();

            //2.com_velocity
            LinearVelocity _com_velocity = robot_state_->getLinearVelocityBaseInWorldFrame();
            com_velocity.resize(3);
            com_velocity[0] = _com_velocity.x();
            com_velocity[1] = _com_velocity.y();
            com_velocity[2] = _com_velocity.z();
            //3.rpy
            com_roll_pitch_yaw.resize(3);
            RotationQuaternion _rotation_orien= robot_state_->getOrientationBaseToWorld().inverted();
            quaternion.resize(4);
            quaternion[0] = _rotation_orien.x();
            quaternion[1] = _rotation_orien.y();
            quaternion[2] = _rotation_orien.z();
            quaternion[3] = _rotation_orien.w();
            EulerAnglesZyx ypr(_rotation_orien);
            Eigen::Vector3d EulerAngle0 = ypr.setUnique().vector();
            com_roll_pitch_yaw[2] = 0;
            //QuaternionToEuler();
            com_roll_pitch_yaw[0] = EulerAngle0(2);
            com_roll_pitch_yaw[1] = EulerAngle0(1);
            //4.ang_vel


           // LocalAngularVelocity _com_angular_velocity =  robot_state_->getAngularVelocityBaseInBaseFrame();

            LocalAngularVelocity _com_angular_velocity =  robot_state_->getAngularVelocityBaseInBaseFrame();
            com_angular_velocity.resize(3);
            com_angular_velocity[0] = _com_angular_velocity.x();
            com_angular_velocity[1] = _com_angular_velocity.y();
            com_angular_velocity[2] = _com_angular_velocity.z();
            //5.contact_states
            foot_contact_states.resize(4);
            for (const auto& leg : limbs_){
                int _n;
                switch (leg) {
                    case free_gait::LimbEnum::LF_LEG:
                        _n = 0;
                        break;
                    case free_gait::LimbEnum::RF_LEG:
                        _n = 1;
                        break;
                    case free_gait::LimbEnum::RH_LEG:
                        _n = 2;
                        break;
                    case free_gait::LimbEnum::LH_LEG:
                        _n = 3;
                        break;
                    default:
                        ROS_ERROR("no leg be chosen");
                }
                if(robot_state_->isSupportLeg(leg)){
                    foot_contact_states[_n] = 1;
                }
                else{
                    foot_contact_states[_n] = 0;
                }
            }
            //6.foot_postion
            foot_positions_body_frame.resize(12);
            for(const auto& leg: limbs_){
                int _n;
                switch (leg) {
                    case free_gait::LimbEnum::LF_LEG:
                        _n = 0;
                        break;
                    case free_gait::LimbEnum::RF_LEG:
                        _n = 1;
                        break;
                    case free_gait::LimbEnum::RH_LEG:
                        _n = 2;
                        break;
                    case free_gait::LimbEnum::LH_LEG:
                        _n = 3;
                        break;
                    default:
                        ROS_ERROR("no leg be chosen");
                }
                Position footpos = robot_state_->getPositionBaseToFootInBaseFrame(leg);
                foot_positions_body_frame[_n * 3]  = footpos.x();
                foot_positions_body_frame[_n * 3 + 1]  = footpos.y();
                foot_positions_body_frame[_n * 3 + 2]  = footpos.z();
            }
        //    std::cout << "Real foot position: " << foot_positions_body_frame[0] <<" " <<
        //                 foot_positions_body_frame[1]<<" "<<
        //                 foot_positions_body_frame[2]<< endl;
            //7.frition_coeffs
            foot_friction_coeffs = {0.45, 0.45, 0.45, 0.45};
            //8.desire_pos
            Position _desired_com_position = robot_state_->getTargetPositionWorldToBaseInWorldFrame();
            desired_com_position.resize(3);
            desired_com_position[0] = 0;
            desired_com_position[1] = 0;
            desired_com_position[2] = _desired_com_position.z();
            //0319
            desired_com_position[2] = 0.36;
            //9.desire_vel
            LinearVelocity _desired_com_velocity = robot_state_->getTargetLinearVelocityBaseInWorldFrame();
            desired_com_velocity.resize(3);
            desired_com_velocity[0] = _desired_com_velocity.x();
            desired_com_velocity[1] = _desired_com_velocity.y();
            desired_com_velocity[2] = 0;
            //10.desire_rpy
            _rotation_orien = robot_state_->getTargetOrientationBaseToWorld();
            desired_quaternion.resize(4);
            desired_quaternion[0] = _rotation_orien.x();
            desired_quaternion[1] = _rotation_orien.y();
            desired_quaternion[2] = _rotation_orien.z();
            desired_quaternion[3] = _rotation_orien.w();
            desired_com_roll_pitch_yaw.resize(3);
            EulerAnglesZyx ypr_d(_rotation_orien);
            Eigen::Vector3d EulerAngle_d=ypr_d.setUnique().vector();
            desired_com_roll_pitch_yaw[2] = 0;
            desired_com_roll_pitch_yaw[0] = 0;
            desired_com_roll_pitch_yaw[1] = 0;
            //11.desire_ang_vel
            LocalAngularVelocity _desired_com_angular_velocity = robot_state_->getTargetAngularVelocityBaseInBaseFrame();
            desired_com_angular_velocity.resize(3);
            desired_com_angular_velocity[0] = 0;
            desired_com_angular_velocity[1] = 0;
            desired_com_angular_velocity[2] = _desired_com_angular_velocity.z();
            return  true;
        }
    }

    bool RosMpcController::computeJointTorques(std::vector<double>& _forces){
        const LinearAcceleration gravitationalAccelerationInWorldFrame = LinearAcceleration(0.0,0.0,-9.8);//torso_->getProperties().getGravity();
        const LinearAcceleration gravitationalAccelerationInBaseFrame = robot_state_->getOrientationBaseToWorld().inverseRotate(gravitationalAccelerationInWorldFrame);//torso_->getMeasuredState().getOrientationWorldToBase().rotate(gravitationalAccelerationInWorldFrame);
        for (auto& legInfo : legInfos_)
        {

          /*
           * Torque setpoints should be updated only is leg is support leg.
           */
          //std::cout << "level1"<<endl;
          if  (robot_state_->isSupportLeg(legInfo.first)) {
              //std::cout << "level2"<<endl;
            if (legInfo.second.isPartOfForceDistribution_ or 1)
            {
                //std::cout << "level3"<<endl;
              Eigen::Matrix3d jacobian = robot_state_->getTranslationJacobianFromBaseToFootInBaseFrame(legInfo.first);
              Force contactForce = legInfo.second.desiredContactForce_;//! WSHY: TODO tranform to hip frame
              int n_leg = static_cast<int>(legInfo.first);
              std::vector<double> _f;
              _f.resize(3);
              for (int i = 0; i < 3; i++) {
                  _f[i] = _forces[i + 3*n_leg];
              }
              //Eigen::Vector3d force_mpc;
              Force contactForce_MPC;
              contactForce_MPC << _f[0], _f[1], _f[2];
      //        ROS_INFO("contact force for %d is : \n", static_cast<int>(legInfo.first));
      //        std::cout<<contactForce.toImplementation()<<std::endl;
              Position tranformed_vector = robot_state_->getPositionFootToHipInHipFrame(legInfo.first, Position(contactForce_MPC.toImplementation()));
              ROS_DEBUG("leg Jacobian for %d is : \n", static_cast<int>(legInfo.first));
              free_gait::JointEffortsLeg jointTorques = free_gait::JointEffortsLeg(jacobian.transpose() * contactForce_MPC.toImplementation());
              free_gait::JointPositionsLeg joint_position_leg = robot_state_->getJointPositionFeedbackForLimb(legInfo.first);
              jointTorques += robot_state_->getGravityCompensationForLimb(legInfo.first, joint_position_leg, free_gait::Force(gravitationalAccelerationInBaseFrame.toImplementation()));
              std::cout << jointTorques << " **";
              robot_state_->setJointEffortsForLimb(legInfo.first, jointTorques);
            }
            else
            {
                robot_state_->setJointEffortsForLimb(legInfo.first, free_gait::JointEffortsLeg::Zero());
            }
          }
        }
        std::cout<<endl;
        return true;
    }

    bool RosMpcController::computeJointTorques(std::vector<double>& _forces, CtrlEnum ways){
        if(ways == CtrlEnum::MIT){
            for (auto& legInfo : legInfos_)
            {

              /*
               * Torque setpoints should be updated only is leg is support leg.
               */
              if  (robot_state_->isSupportLeg(legInfo.first)) {
                if (legInfo.second.isPartOfForceDistribution_ or 1)
                {
                  Eigen::Matrix3d jacobian = robot_state_->getTranslationJacobianFromBaseToFootInBaseFrame(legInfo.first);
                  int n_leg = static_cast<int>(legInfo.first);
                  std::vector<double> _f;
                  _f.resize(3);
                  for (int i = 0; i < 3; i++) {
                      _f[i] = _forces[i + 3*n_leg];
                  }
                  //Eigen::Vector3d force_mpc;
                  Force contactForce_MPC;
                  contactForce_MPC << _f[0], _f[1], _f[2];
                  free_gait::JointEffortsLeg jointTorques = free_gait::JointEffortsLeg(jacobian.transpose() * contactForce_MPC.toImplementation());
                  robot_state_->setJointEffortsForLimb(legInfo.first, jointTorques);
                }
                else
                {
                    robot_state_->setJointEffortsForLimb(legInfo.first, free_gait::JointEffortsLeg::Zero());
                }
              }
            }
            return true;
        }
    }
    bool RosMpcController::loadParams_MPC(const ros::NodeHandle& node_handle){
        if (node_handle.hasParam("/mpc/weights")) {
          node_handle.getParam("/mpc/weights", _MPC_WEIGHTS);
        } else {
          ROS_ERROR("Did not find ROS parameter for robot state topic '/mpc/weights'.");
          return false;
        }
//        if (node_handle.hasParam("/mpc/roll")) {
//          node_handle.getParam("/mpc/roll", _MPC_WEIGHTS[0]);
//        } else {
//          ROS_ERROR("Did not find ROS parameter for robot state topic '/mpc/roll'.");
//          return false;
//        }
//        if (node_handle.hasParam("/mpc/pitch")) {
//          node_handle.getParam("/mpc/pitch", _MPC_WEIGHTS[1]);
//        } else {
//          ROS_ERROR("Did not find ROS parameter for robot state topic '/mpc/pitch'.");
//          return false;
//        }
//        if (node_handle.hasParam("/mpc/yaw")) {
//          node_handle.getParam("/mpc/yaw", _MPC_WEIGHTS[2]);
//        } else {
//          ROS_ERROR("Did not find ROS parameter for robot state topic '/mpc/yaw'.");
//          return false;
//        }
//        if (node_handle.hasParam("/mpc/x")) {
//          node_handle.getParam("/mpc/x", _MPC_WEIGHTS[3]);
//        } else {
//          ROS_ERROR("Did not find ROS parameter for robot state topic '/mpc/x'.");
//          return false;
//        }
//        if (node_handle.hasParam("/mpc/y")) {
//          node_handle.getParam("/mpc/y", _MPC_WEIGHTS[4]);
//        } else {
//          ROS_ERROR("Did not find ROS parameter for robot state topic '/mpc/y'.");
//          return false;
//        }
//        if (node_handle.hasParam("/mpc/z")) {
//          node_handle.getParam("/mpc/z", _MPC_WEIGHTS[5]);
//        } else {
//          ROS_ERROR("Did not find ROS parameter for robot state topic '/mpc/z'.");
//          return false;
//        }
//        if (node_handle.hasParam("/mpc/roll_vel")) {
//          node_handle.getParam("/mpc/roll_vel", _MPC_WEIGHTS[6]);
//        } else {
//          ROS_ERROR("Did not find ROS parameter for robot state topic '/mpc/roll_vel'.");
//          return false;
//        }
//        if (node_handle.hasParam("/mpc/pitch_vel")) {
//          node_handle.getParam("/mpc/pitch_vel", _MPC_WEIGHTS[7]);
//        } else {
//          ROS_ERROR("Did not find ROS parameter for robot state topic '/mpc/pitch_vel'.");
//          return false;
//        }
//        if (node_handle.hasParam("/mpc/yaw_vel")) {
//          node_handle.getParam("/mpc/yaw_vel", _MPC_WEIGHTS[8]);
//        } else {
//          ROS_ERROR("Did not find ROS parameter for robot state topic '/mpc/yaw_vel'.");
//          return false;
//        }
//        if (node_handle.hasParam("/mpc/x_vel")) {
//          node_handle.getParam("/mpc/x_vel", _MPC_WEIGHTS[9]);
//        } else {
//          ROS_ERROR("Did not find ROS parameter for robot state topic '/mpc/x_vel'.");
//          return false;
//        }
//        if (node_handle.hasParam("/mpc/y_vel")) {
//          node_handle.getParam("/mpc/y_vel", _MPC_WEIGHTS[10]);
//        } else {
//          ROS_ERROR("Did not find ROS parameter for robot state topic '/mpc/y_vel'.");
//          return false;
//        }
//        if (node_handle.hasParam("/mpc/z_vel")) {
//          node_handle.getParam("/mpc/z_vel", _MPC_WEIGHTS[11]);
//        } else {
//          ROS_ERROR("Did not find ROS parameter for robot state topic '/mpc/z_vel'.");
//          return false;
//        }
//        if (node_handle.hasParam("/mpc/gravity_holder")) {
//          node_handle.getParam("/mpc/gravity_holder", _MPC_WEIGHTS[12]);
//        } else {
//          ROS_ERROR("Did not find ROS parameter for robot state topic '/mpc/gravity_holder'.");
//          return false;
//        }
        if (node_handle.hasParam("/mpc/steps_MPC")) {
          node_handle.getParam("/mpc/steps_MPC", steps_MPC);
        } else {
          ROS_ERROR("Did not find ROS parameter for robot state topic '/mpc/steps_MPC'.");
          return false;
        }
        if (node_handle.hasParam("/mpc/delta_t_MPC")) {
          node_handle.getParam("/mpc/delta_t_MPC", delta_t_MPC);
        } else {
          ROS_ERROR("Did not find ROS parameter for robot state topic '/mpc/delta_t_MPC'.");
          return false;
        }
        if (node_handle.hasParam("/mpc/torque_weight")) {
          node_handle.getParam("/mpc/torque_weight", torque_weight);
        } else {
          ROS_ERROR("Did not find ROS parameter for robot state topic '/mpc/torque_weight'.");
          return false;
        }
        return true;
    }

    bool RosMpcController::logDataCapture(std_srvs::Empty::Request& req,
                        std_srvs::Empty::Response& res)
    {
      ROS_INFO("Call to Capture Log Data");
      for(int index = 0; index<desired_robot_state_.size(); index++)
        {
          base_command_pub_.publish(base_command_pose_[index]);
          base_actual_pub_.publish(base_actual_pose_[index]);
          leg_state_pub_.publish(leg_states_[index]);
          joint_command_pub_.publish(joint_command_[index]);
          joint_actual_pub_.publish(joint_actual_[index]);
          contact_desired_pub_.publish(foot_desired_contact_[index]);
          leg_phase_pub_.publish(leg_phases_[index]);
          desired_robot_state_pub_.publish(desired_robot_state_[index]);
          actual_robot_state_pub_.publish(actual_robot_state_[index]);
          vmc_info_pub_.publish(vitual_force_torque_[index]);
          desired_vmc_info_pub_.publish(desired_vitual_force_torque_[index]);
          motor_status_word_pub_.publish(motor_status_word_[index]);
          ros::Duration(0.0025).sleep();
        }
      return true;
    }

};

PLUGINLIB_EXPORT_CLASS(balance_controller::RosMpcController, controller_interface::ControllerBase)
