#include <robot_state_lcm_hardware_interface.hpp>


namespace laikago_ros_control {

using namespace laikago;

//! Must set control level here
Control control(LOWLEVEL);
float Kv[3] = {0};
float Kp[3] = {0};
static long motiontime = 0;
RobotStateLcmHardwareInterface::RobotStateLcmHardwareInterface()
{
    //Publish IMU data for stata estimation.
    Imu_data_pub_ = node_handle_.advertise<sensor_msgs::Imu>("/imu", 1);
    //Publish joint state for showing on RVIZ.
    joint_state_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/joint_states", 1);
    lf_foot_contact_force_pub = node_handle_.advertise<geometry_msgs::WrenchStamped>("/lf_contact_force",1);
    rf_foot_contact_force_pub = node_handle_.advertise<geometry_msgs::WrenchStamped>("/rf_contact_force",1);
    rh_foot_contact_force_pub = node_handle_.advertise<geometry_msgs::WrenchStamped>("/rh_contact_force",1);
    laikago_position_init_server_ = node_handle_.advertiseService("/laikago_position_init", &RobotStateLcmHardwareInterface::Position_initCB,this);
    laikago_position_init_stop_server_ = node_handle_.advertiseService("/laikago_position_init_stop", &RobotStateLcmHardwareInterface::Position_stopCB,this);
    laikago_controller_switch_server_ = node_handle_.advertiseService("/mxr/switch_controller",&RobotStateLcmHardwareInterface::Controller_switchCB,this);
    lh_foot_contact_force_pub = node_handle_.advertise<geometry_msgs::WrenchStamped>("/lh_contact_force",1);
    ROS_INFO("Build Lcm Hardware Interface ");
}

RobotStateLcmHardwareInterface::~RobotStateLcmHardwareInterface()
{
    ROS_INFO("Lcm Hardware Interface Shutdown");
}

bool RobotStateLcmHardwareInterface::loadParameters(ros::NodeHandle &nh)
{
    if(!nh.getParam("/imu_topic_name", imu_topic_name_))
    {
        ROS_ERROR("Can not load IMU topic name ");
        return false;
    }
}

bool RobotStateLcmHardwareInterface::Position_initCB(std_srvs::SetBool::Request& request,
                                std_srvs::SetBool::Response& response)
{
    if(request.data == true){
        laikago_position_init_buffer_ = true;
        ROS_INFO("Start joint position init!!!!!!!!!!!!!!");
    }
    if(request.data == false){
        laikago_position_init_buffer_ = false;
        ROS_INFO("STOP Start joint position....");
      }
  return true;
}
bool RobotStateLcmHardwareInterface::Position_stopCB(std_srvs::SetBool::Request& request,
                                std_srvs::SetBool::Response& response)
{
    if(request.data == true){
        laikago_position_init_buffer_ = false;
        ROS_INFO("Stop joint position init!!!!!!!!!!!!!!");
    }
    if(request.data == false){
        laikago_position_init_buffer_ = true;
        ROS_INFO("Start joint position....");
      }
  return true;
}

bool RobotStateLcmHardwareInterface::Controller_switchCB(controller_manager_msgs::SwitchController::Request& request,
                                controller_manager_msgs::SwitchController::Response& response)
{
    //if(request.stop_controllers.data())

    controller_name = request.start_controllers.back();
//    if(request.data == true){
//        laikago_position_init_buffer_ = false;
//        ROS_INFO("Stop joint position init!!!!!!!!!!!!!!");
//    }
//    if(request.data == false){
//        laikago_position_init_buffer_ = true;
//        ROS_INFO("Start joint position....");
//      }
  return true;
}
bool RobotStateLcmHardwareInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
{
    ROS_INFO("Initializing RobotStateLcmHardwareInterface");
    node_handle_ = root_nh;

    //! WSHY: initial data of robot state
    robot_state_data_.name = "base_controller";
    robot_state_data_.position = position;
    robot_state_data_.orientation = orinetation;
    robot_state_data_.linear_velocity = linear_vel;
    robot_state_data_.angular_velocity = angular_vel;
    robot_state_data_.joint_position_read = pos_read;
    robot_state_data_.joint_position_write = pos_write;
    robot_state_data_.joint_velocity_read = vel_read;
    robot_state_data_.joint_velocity_write = vel_write;
    robot_state_data_.joint_effort_read = eff_read;
    robot_state_data_.joint_effort_write = eff_write;
    robot_state_data_.foot_contact = foot_contact;
    robot_state_data_.contact_pressure = contact_pressure;
    robot_state_data_.motor_status_word = motor_status_word;
    robot_state_data_.mode_of_joint = mode_of_joint;
    //! WSHY: registerhandle pass the data point to the hardwareResourseManager and then
    //! the read() method update data which the pointer points to or write() the
    //! updated commmand
    robot_state_interface_.registerHandle(hardware_interface::RobotStateHandle(robot_state_data_));

    std::string urdf_string;
    if(!root_nh.getParam("/robot_description", urdf_string))
      {
        ROS_ERROR("Failed to load urdf from robot_descriptions");
        return false;
      }
    transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);
    urdf::Model urdf_model;
    urdf_model.initString(urdf_string);
    const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;
    std::string param_name = "/base_balance_controller/joints";
    if(!root_nh.getParam(param_name, joint_names_))
      {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << root_nh.getNamespace() << ").");
        return false;
      }


    n_dof_ = joint_names_.size();
    std::cout << "number is " << n_dof_ << std::endl;

    // Resize vectors to our DOF
    joint_names_.resize(n_dof_);
    joint_types_.resize(n_dof_);
    joint_lower_limits_.resize(n_dof_);
    joint_upper_limits_.resize(n_dof_);
    joint_effort_limits_.resize(n_dof_);
    joint_control_methods_.resize(n_dof_);
    last_joint_control_methods_.resize(n_dof_);
    pid_controllers_.resize(n_dof_);
    joint_position_.resize(n_dof_);
    joint_velocity_.resize(n_dof_);
    joint_effort_.resize(n_dof_);
    joint_effort_command_.resize(n_dof_);
    joint_position_command_.resize(n_dof_);
    joint_velocity_command_.resize(n_dof_);

    // Initialize values
    const ros::NodeHandle joint_limit_nh(root_nh);

    hardware_interface::JointHandle joint_handle;

    //std::cout<<transmissions_[j].joints_.size()<<transmissions_[j].joints_.size()<<std::endl;
    for(unsigned int j = 0;j<n_dof_;j++)
    {
        // Check that this transmission has one joint
        if(transmissions_[j].joints_.size() == 0)
        {
          ROS_WARN_STREAM_NAMED("ros_ethercat_hardware_interface","Transmission " << transmissions_[j].name_
            << " has no associated joints.");
          continue;
        }
        else if(transmissions_[j].joints_.size() > 1)
        {
          ROS_WARN_STREAM_NAMED("ros_ethercat_hardware_interface","Transmission " << transmissions_[j].name_
            << " has more than one joint. Currently the default robot hardware simulation "
            << " interface only supports one.");
          continue;
        }

        std::vector<std::string> joint_interfaces = transmissions_[j].joints_[0].hardware_interfaces_;
        if (joint_interfaces.empty() &&
            !(transmissions_[j].actuators_.empty()) &&
            !(transmissions_[j].actuators_[0].hardware_interfaces_.empty()))
        {
          // TODO: Deprecate HW interface specification in actuators in ROS J
          joint_interfaces = transmissions_[j].actuators_[0].hardware_interfaces_;
          ROS_WARN_STREAM_NAMED("ros_ethercat_hardware_interface", "The <hardware_interface> element of tranmission " <<
            transmissions_[j].name_ << " should be nested inside the <joint> element, not <actuator>. " <<
            "The transmission will be properly loaded, but please update " <<
            "your robot model to remain compatible with future versions of the plugin.");
        }
        if (joint_interfaces.empty())
        {
          ROS_WARN_STREAM_NAMED("ros_ethercat_hardware_interface", "Joint " << transmissions_[j].joints_[0].name_ <<
            " of transmission " << transmissions_[j].name_ << " does not specify any hardware interface. " <<
            "Not adding it to the robot hardware simulation.");
          continue;
        }
        else if (joint_interfaces.size() > 1)
        {
          ROS_WARN_STREAM_NAMED("ros_ethercat_hardware_interface", "Joint " << transmissions_[j].joints_[0].name_ <<
            " of transmission " << transmissions_[j].name_ << " specifies multiple hardware interfaces. " <<
            "Currently the default robot hardware simulation interface only supports one. Using the first entry");
          //continue;
        }
        ROS_INFO("Joint name is ====================== %s", joint_names_[j].c_str());
        const std::string& hardware_interface = joint_interfaces.front();

                if(j==0||j==9){
                    joint_position_[j]=0.2;
                }
                if(j==3||j==6){
                    joint_position_[j]=-0.2;
                }
                if(j==1||j==4||j==7||j==10){
                    joint_position_[j]=1.8;
                }
                if(j==2||j==5||j==8||j==11){
                    joint_position_[j]=-2.7;
                }
        //joint_position_[j] = 0.0;
        joint_velocity_[j] = 0.0;
        joint_effort_[j] = 0.0;  // N/m for continuous joints
        joint_effort_command_[j] = 0.0;
        joint_position_command_[j] = 0.0;
        joint_velocity_command_[j] = 0.0;

        //ROS_INFO("Joint name is ====================== %s", joint_names_[j].c_str());

        // Create joint state interface for all joints
        js_interface_.registerHandle(hardware_interface::JointStateHandle(
            joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));


        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                       &joint_position_command_[j]);
        pj_interface_.registerHandle(joint_handle);
        robot_state_interface_.joint_position_interfaces_.registerHandle(joint_handle);

        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                       &joint_effort_command_[j]);
        ej_interface_.registerHandle(joint_handle);
        robot_state_interface_.joint_effort_interfaces_.registerHandle(joint_handle);





        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                       &joint_velocity_command_[j]);
        vj_interface_.registerHandle(joint_handle);
        //        std::cout<<"#############################"<<std::endl;
        //        std::cout<<joint_position_command_[j]<<std::endl;
        //        std::cout<<"#############################"<<std::endl;
    }

    // Register interfaces
    //! WSHY: the controller pass the interface in it
    registerInterface(&js_interface_);
    registerInterface(&ej_interface_);
    registerInterface(&pj_interface_);
    registerInterface(&vj_interface_);
    std::cout << "register" <<std::endl;
  //  registerInterface(&imu_interface_);
    registerInterface(&robot_state_interface_);

    // Initialize the emergency stop code.
    e_stop_active_ = false;
    last_e_stop_active_ = false;

    // Lcm subscribe robot state
    roslcm.SubscribeState();
    // update loop
    update_thread_ = boost::thread(boost::bind(&RobotStateLcmHardwareInterface::update_loop, this));
    // Set control level
    SendLowROS.levelFlag = laikago::LOWLEVEL;
    for(int i = 1; i < 13; i++)
    {
        SendLowROS.motorCmd[i].mode = 0x0A;
    }

    ROS_INFO("Successfully Initialize Lcm Hardware Interface");

    return true;
  }

void RobotStateLcmHardwareInterface::update_loop()
{
    while(ros::ok())
    {
        //ROS_INFO("update loop");
        boost::mutex::scoped_lock lock(mutex);
        //ROS_INFO("Receive robot state info ");
        roslcm.Recv();
        lock.unlock();
        roslcm.Get(RecvLowLCM);
        memcpy(&RecvLowROS, &RecvLowLCM, sizeof (laikago::LowState));

        imu_msgs_ = getImuMsgs(RecvLowROS);
        Imu_data_pub_.publish(imu_msgs_);
        getFootForces(RecvLowROS);

        joint_state_msgs_ = getJointStateMsgs(RecvLowROS);
        joint_state_pub_.publish(joint_state_msgs_);
        lf_foot_contact_force_pub.publish(lf_contact_force);
        rf_foot_contact_force_pub.publish(rf_contact_force);
        rh_foot_contact_force_pub.publish(rh_contact_force);
        lh_foot_contact_force_pub.publish(lh_contact_force);
        usleep(2000); // 500HZ
    }

}

void RobotStateLcmHardwareInterface::getFootForces(laikago_msgs::LowState &low_state){
    rf_contact_force.wrench.force.z = low_state.footForce[0];
    lf_contact_force.wrench.force.z = low_state.footForce[1];
    rh_contact_force.wrench.force.z = low_state.footForce[2];
    lh_contact_force.wrench.force.z = low_state.footForce[3];
}
const sensor_msgs::Imu RobotStateLcmHardwareInterface::getImuMsgs(laikago_msgs::LowState &low_state)
{
    sensor_msgs::Imu imu_msgs;
    //orientation data conversion
    imu_msgs.header.frame_id = "imu_link";
    imu_msgs.header.stamp = ros::Time::now();
    imu_msgs.orientation.w = low_state.imu.quaternion[0];
    imu_msgs.orientation.x = low_state.imu.quaternion[1];
    imu_msgs.orientation.y = low_state.imu.quaternion[2];
    imu_msgs.orientation.z = low_state.imu.quaternion[3];
    //angular velocity data conversion
    imu_msgs.angular_velocity.x = low_state.imu.gyroscope[0];
    //ROS_INFO_STREAM("velocity x: " << imu_msgs.angular_velocity.x << std::endl);
    imu_msgs.angular_velocity.y = low_state.imu.gyroscope[1];
    //ROS_INFO_STREAM("velocity y: " << imu_msgs.angular_velocity.y << std::endl);
    imu_msgs.angular_velocity.z = low_state.imu.gyroscope[2];
    //ROS_INFO_STREAM("angular velocity z: " << imu_msgs.angular_velocity.z << std::endl);
    //linear acceleration data conversion
    imu_msgs.linear_acceleration.x = low_state.imu.acceleration[0];
    imu_msgs.linear_acceleration.y = low_state.imu.acceleration[1];
    imu_msgs.linear_acceleration.z = low_state.imu.acceleration[2];

    return imu_msgs;
}

const sensor_msgs::JointState RobotStateLcmHardwareInterface::getJointStateMsgs(laikago_msgs::LowState &lowState)
{
    sensor_msgs::JointState joint_state;
    joint_state.header.frame_id = "base";
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(12);
    //previous joint name for laikago
//    joint_state.name[0] = "FL_hip_joint";
//    joint_state.name[1] = "FL_thigh_joint";
//    joint_state.name[2] = "FL_calf_joint";
//    joint_state.name[3] = "FR_hip_joint";
//    joint_state.name[4] = "FR_thigh_joint";
//    joint_state.name[5] = "FR_calf_joint";
//    joint_state.name[6] = "RL_hip_joint";
//    joint_state.name[7] = "RL_thigh_joint";
//    joint_state.name[8] = "RL_calf_joint";
//    joint_state.name[9] = "RR_hip_joint";
//    joint_state.name[10] = "RR_thigh_joint";
//    joint_state.name[11] = "RR_calf_joint";

    joint_state.name[0] = "LF_HAA";
    joint_state.name[1] = "LF_HFE";
    joint_state.name[2] = "LF_KFE";
    joint_state.name[3] = "LH_HAA";
    joint_state.name[4] = "LH_HFE";
    joint_state.name[5] = "LH_KFE";
    joint_state.name[6] = "RF_HAA";
    joint_state.name[7] = "RF_HFE";
    joint_state.name[8] = "RF_KFE";
    joint_state.name[9] = "RH_HAA";
    joint_state.name[10] = "RH_HFE";
    joint_state.name[11] = "RH_KFE";

    joint_state.position.resize(12);
    joint_state.position[0] = lowState.motorState[FL_0].position;
    joint_state.position[1] = lowState.motorState[FL_1].position;
    joint_state.position[2] = lowState.motorState[FL_2].position;
    joint_state.position[6] = lowState.motorState[FR_0].position;
    joint_state.position[7] = lowState.motorState[FR_1].position;
    joint_state.position[8] = lowState.motorState[FR_2].position;
    joint_state.position[3] = lowState.motorState[RL_0].position;
    joint_state.position[4] = lowState.motorState[RL_1].position;
    joint_state.position[5] = lowState.motorState[RL_2].position;
    joint_state.position[9] = lowState.motorState[RR_0].position;
    joint_state.position[10] = lowState.motorState[RR_1].position;
    joint_state.position[11] = lowState.motorState[RR_2].position;

    joint_state.velocity.resize(12);
    joint_state.velocity[0] = lowState.motorState[FL_0].velocity;
    joint_state.velocity[1] = lowState.motorState[FL_1].velocity;
    joint_state.velocity[2] = lowState.motorState[FL_2].velocity;
    joint_state.velocity[6] = lowState.motorState[FR_0].velocity;
    joint_state.velocity[7] = lowState.motorState[FR_1].velocity;
    joint_state.velocity[8] = lowState.motorState[FR_2].velocity;
    joint_state.velocity[3] = lowState.motorState[RL_0].velocity;
    joint_state.velocity[4] = lowState.motorState[RL_1].velocity;
    joint_state.velocity[5] = lowState.motorState[RL_2].velocity;
    joint_state.velocity[9] = lowState.motorState[RR_0].velocity;
    joint_state.velocity[10] = lowState.motorState[RR_1].velocity;
    joint_state.velocity[11] = lowState.motorState[RR_2].velocity;


    joint_state.effort.resize(12);
    joint_state.effort[0] = lowState.motorState[FL_0].torque;
    joint_state.effort[1] = lowState.motorState[FL_1].torque;
    joint_state.effort[2] = lowState.motorState[FL_2].torque;
    joint_state.effort[6] = lowState.motorState[FR_0].torque;
    joint_state.effort[7] = lowState.motorState[FR_1].torque;
    joint_state.effort[8] = lowState.motorState[FR_2].torque;
    joint_state.effort[3] = lowState.motorState[RL_0].torque;
    joint_state.effort[4] = lowState.motorState[RL_1].torque;
    joint_state.effort[5] = lowState.motorState[RL_2].torque;
    joint_state.effort[9] = lowState.motorState[RR_0].torque;
    joint_state.effort[10] = lowState.motorState[RR_1].torque;
    joint_state.effort[11] = lowState.motorState[RR_2].torque;

    return joint_state;
}

void RobotStateLcmHardwareInterface::read(const ros::Time &time, const ros::Duration &period)
{
    //ROS_INFO("READ ONCE");
    boost::recursive_mutex::scoped_lock lock(r_mutex_);
    //roslcm.Recv();
    //roslcm.Get(RecvLowLCM);
    //memcpy(&RecvLowROS, &RecvLowLCM, sizeof (laikago::LowState));

    //roslcm.Get(RecvHighLCM);
    //memcpy(&RecvHighROS, &RecvHighLCM, sizeof (laikago::HighState));


    //read joint position
    //printf("%f\n",  RecvLowROS.motorState[FL_2].position);
    robot_state_data_.joint_position_read[0] = RecvLowROS.motorState[FL_0].position;
    //ROS_WARN_STREAM("joint position: " << robot_state_data_.joint_position_read[1]);
    robot_state_data_.joint_position_read[1] = RecvLowROS.motorState[FL_1].position;
    robot_state_data_.joint_position_read[2] = RecvLowROS.motorState[FL_2].position;
    //ROS_WARN_STREAM("joint position: " << robot_state_data_.joint_position_read[2]);
    robot_state_data_.joint_position_read[3] = RecvLowROS.motorState[FR_0].position;
    robot_state_data_.joint_position_read[4] = RecvLowROS.motorState[FR_1].position;
    robot_state_data_.joint_position_read[5] = RecvLowROS.motorState[FR_2].position;
    robot_state_data_.joint_position_read[6] = RecvLowROS.motorState[RR_0].position;
    robot_state_data_.joint_position_read[7] = RecvLowROS.motorState[RR_1].position;
    robot_state_data_.joint_position_read[8] = RecvLowROS.motorState[RR_2].position;
    robot_state_data_.joint_position_read[9] = RecvLowROS.motorState[RL_0].position;
    robot_state_data_.joint_position_read[10] = RecvLowROS.motorState[RL_1].position;
    robot_state_data_.joint_position_read[11] = RecvLowROS.motorState[RL_2].position;
    //read joint velocity
    robot_state_data_.joint_velocity_read[0] = RecvLowROS.motorState[FL_0].velocity;
    robot_state_data_.joint_velocity_read[1] = RecvLowROS.motorState[FL_1].velocity;
    robot_state_data_.joint_velocity_read[2] = RecvLowROS.motorState[FL_2].velocity;
    robot_state_data_.joint_velocity_read[3] = RecvLowROS.motorState[FR_0].velocity;
    robot_state_data_.joint_velocity_read[4] = RecvLowROS.motorState[FR_1].velocity;
    robot_state_data_.joint_velocity_read[5] = RecvLowROS.motorState[FR_2].velocity;
    robot_state_data_.joint_velocity_read[6] = RecvLowROS.motorState[RR_0].velocity;
    robot_state_data_.joint_velocity_read[7] = RecvLowROS.motorState[RR_1].velocity;
    robot_state_data_.joint_velocity_read[8] = RecvLowROS.motorState[RR_2].velocity;
    robot_state_data_.joint_velocity_read[9] = RecvLowROS.motorState[RL_0].velocity;
    robot_state_data_.joint_velocity_read[10] = RecvLowROS.motorState[RL_1].velocity;
    robot_state_data_.joint_velocity_read[11] = RecvLowROS.motorState[RL_2].velocity;
    //read joint effort
    robot_state_data_.joint_effort_read[0] = RecvLowROS.motorState[FL_0].torque;
    robot_state_data_.joint_effort_read[1] = RecvLowROS.motorState[FL_1].torque;
    robot_state_data_.joint_effort_read[2] = RecvLowROS.motorState[FL_2].torque;
    robot_state_data_.joint_effort_read[3] = RecvLowROS.motorState[FR_0].torque;
    robot_state_data_.joint_effort_read[4] = RecvLowROS.motorState[FR_1].torque;
    robot_state_data_.joint_effort_read[5] = RecvLowROS.motorState[FR_2].torque;
    robot_state_data_.joint_effort_read[6] = RecvLowROS.motorState[RR_0].torque;
    robot_state_data_.joint_effort_read[7] = RecvLowROS.motorState[RR_1].torque;
    robot_state_data_.joint_effort_read[8] = RecvLowROS.motorState[RR_2].torque;
    robot_state_data_.joint_effort_read[9] = RecvLowROS.motorState[RL_0].torque;
    robot_state_data_.joint_effort_read[10] = RecvLowROS.motorState[RL_1].torque;
    robot_state_data_.joint_effort_read[11] = RecvLowROS.motorState[RL_2].torque;
    //read foot contact state
   robot_state_data_.contact_pressure[1] = RecvLowROS.footForce[0];
   robot_state_data_.contact_pressure[0] = RecvLowROS.footForce[1];
   robot_state_data_.contact_pressure[2] = RecvLowROS.footForce[2];
   robot_state_data_.contact_pressure[3] = RecvLowROS.footForce[3];


    lock.unlock();
}

bool RobotStateLcmHardwareInterface::Init_flag(){
    if(joint_position_command_[0]==0.2&&joint_position_command_[1]==1.8&&joint_position_command_[2]==-2.7&&
            joint_position_command_[3]==-0.2&&joint_position_command_[4]==1.8&&joint_position_command_[5]==-2.7&&
            joint_position_command_[6]==-0.2&&joint_position_command_[7]==1.8&&joint_position_command_[8]==-2.7&&
            joint_position_command_[9]==0.2&&joint_position_command_[10]==1.8&&joint_position_command_[11]==-2.7){
        return true;
    }

    return false;
}
void RobotStateLcmHardwareInterface::write(const ros::Time &time, const ros::Duration &period)
{
    //ROS_INFO("Write Once");
    boost::recursive_mutex::scoped_lock lock(r_mutex_);
    double currentPos, currentVel, calcTorque;
    torque_set.resize(12);

    //MXR::NOTE: currently for reset(todo:button?)
//    if(joint_position_command_[0]==0.5||joint_position_command_[1]==1.8||joint_position_command_[2]==-2.7){
//        motiontime = 0;
//    }

    //MXR:NOTE:  if we check the joint command,the joint_position_command is {0,0.67,-1.3}
//        for(unsigned int j = 0;j<n_dof_;j++)
//        {


//            if(joint_position_command_[j]!=0||joint_effort_command_[j]!=0||joint_velocity_command_[j]!=0){
//                std::cout<<"#############################"<<std::endl;
//                std::cout<<controller_name<<std::endl;
//                std::cout<<j<<"  "<<joint_position_command_[j]<<"  "<<joint_effort_command_[j]<<"  "<<joint_velocity_command_[j]<<std::endl;
//                std::cout<<"#############################"<<std::endl;
//            }


//        }



//    //MXR::NOTE: take the laikago like the laikago_joint_control but the compute torque isn't very good
//    for(unsigned int j = 0;j<n_dof_;j++)
//    {
//        //last_joint_position_command_.resize(12);
//        //last_joint_position_command_[j] = joint_position_command_[j];
////        std::cout<<"#############################"<<std::endl;
////        std::cout<<joint_position_command_[j]<<"  "<<joint_effort_command_[j]<<"  "<<joint_velocity_command_[j]<<std::endl;
////        std::cout<<"#############################"<<std::endl;

//        servoCmd.pos = joint_position_command_[j];
//        servoCmd.vel = joint_velocity_command_[j];
//        servoCmd.torque  = joint_effort_command_[j];

//        if(j==0||j==3||j==6||j==9){
//            servoCmd.posStiffness = Kp[0];
//            servoCmd.velStiffness = Kv[0];
//        }
//        if(j==1||j==4||j==7||j==10){
//            servoCmd.posStiffness = Kp[1];
//            servoCmd.velStiffness = Kv[1];
//        }
//        if(j==2||j==5||j==8||j==11){
//            servoCmd.posStiffness = Kp[2];
//            servoCmd.velStiffness = Kv[2];
//        }
//        currentPos = robot_state_data_.joint_position_read[j];
//        if(multi_lastState.size()!=12){
//            for(unsigned int m = 0;m<n_dof_;m++){
//                multi_lastState.resize(12);
//                multi_lastState[m].position = robot_state_data_.joint_position_read[m];
//                multi_lastState[m].velocity = 0.0;
//            }
//        }
//        currentVel = computeVel(currentPos, multi_lastState[j].position,multi_lastState[j].velocity, period.toSec());
//        calcTorque = computeTorque(currentPos, currentVel, servoCmd);
//        torque_set[j] = calcTorque;

////        std::cout<<"+++++++++++++++++++++++++++++++++"<<std::endl;
////        std::cout<<currentPos<<" "<<currentVel<<" "<<calcTorque<<std::endl;
////        std::cout<<"+++++++++++++++++++++++++++++++++"<<std::endl;

//        multi_servoCmd.push_back(servoCmd);

//        if(j==0||j==3||j==6||j==9){
//            HAAeffortLimits(servoCmd.torque);
//        }
//        if(j==1||j==4||j==7||j==10){
//            HFEeffortLimits(servoCmd.torque);
//        }
//        if(j==2||j==5||j==8||j==11){
//            KFEeffortLimits(servoCmd.torque);
//        }
//        if(j==0||j==3||j==6||j==9){
//            HAAeffortLimits(calcTorque);
//        }
//        if(j==1||j==4||j==7||j==10){
//            HFEeffortLimits(calcTorque);
//        }
//        if(j==2||j==5||j==8||j==11){
//            KFEeffortLimits(calcTorque);
//        }



//    }
//    multi_lastState.clear();
//    for(unsigned int k = 0;k<n_dof_;k++)
//    {
//        lastState.position = robot_state_data_.joint_position_read[k];
//        lastState.velocity = robot_state_data_.joint_velocity_read[k];
//        multi_lastState.push_back(lastState);
//    }
//         std::cout<<"multi_servoCmd size: "<<multi_servoCmd.size()<<std::endl;
//         std::cout<<"multi_state size: "<<multi_lastState.size()<<std::endl;


        if(last_laikago_position_init_buffer_==false&&laikago_position_init_buffer_==true){
            motiontime=0;
            test_flag=true;
        }
        last_laikago_position_init_buffer_= laikago_position_init_buffer_;
    //MXR::NOTE: better to have a slow change of positionStiffness and velocityStiffness
//###########################(position_control)##########################################
       // MXR::NOTE: for position init


//                std::cout<<"#############################"<<std::endl;
//                std::cout<<controller_name<<std::endl;
//                std::cout<<Init_flag()<<std::endl;
//                std::cout<<"#############################"<<std::endl;
    motiontime++;
    if( motiontime >= 50&&Init_flag()&&(controller_name==""||controller_name=="all_joints_position_group_controller")){


        //增大速度刚度,温柔的调狗
        if( motiontime == 50){
            Kp[0] = 0.132; Kp[1] = 0.032; Kp[2] = 0.032;
            Kv[0] = 0.16; Kv[1] = 0.16; Kv[2] = 0.16;
        }

        if( motiontime == 1000){
            Kp[0] = 0.52; Kp[1] = 0.062; Kp[2] = 0.062;
            Kv[0] = 0.12; Kv[1] = 0.12; Kv[2] = 0.12;
        }

        if( motiontime == 1200){
            Kp[0] = 0.92; Kp[1] = 0.092; Kp[2] = 0.092;
            Kv[0] = 0.06; Kv[1] = 0.06; Kv[2] = 0.06;
        }

        if( motiontime == 2000){
            Kp[0] = 1.32; Kp[1] = 0.132; Kp[2] = 0.132;
            Kv[0] = 0.02; Kv[1] = 0.02; Kv[2] = 0.02;
        }

        if( motiontime == 2500){
            Kp[0] = 1.84; Kp[1] = 0.263; Kp[2] = 0.263;
            Kv[0] = 0.02; Kv[1] = 0.02; Kv[2] = 0.02;
        }

        if( motiontime == 3000){
            Kp[0] = 2.34; Kp[1] = 0.693; Kp[2] = 0.693;
            Kv[0] = 0.02; Kv[1] = 0.02; Kv[2] = 0.02;
        }


        SendLowROS.motorCmd[FR_0].position = joint_position_command_[3];
        SendLowROS.motorCmd[FR_0].velocity = 0;
        SendLowROS.motorCmd[FR_0].positionStiffness = Kp[0];
        SendLowROS.motorCmd[FR_0].velocityStiffness = Kv[0];
        SendLowROS.motorCmd[FR_0].torque = -0.65f;

        SendLowROS.motorCmd[FR_1].position = joint_position_command_[4];
        SendLowROS.motorCmd[FR_1].velocity = 0;
        SendLowROS.motorCmd[FR_1].positionStiffness = Kp[1];
        SendLowROS.motorCmd[FR_1].velocityStiffness = Kv[1];
        SendLowROS.motorCmd[FR_1].torque = 0.0f;

        SendLowROS.motorCmd[FR_2].position =  joint_position_command_[5];
        SendLowROS.motorCmd[FR_2].velocity = 0;
        SendLowROS.motorCmd[FR_2].positionStiffness = Kp[2];
        SendLowROS.motorCmd[FR_2].velocityStiffness = Kv[2];
        SendLowROS.motorCmd[FR_2].torque = 0.0f;

        SendLowROS.motorCmd[FL_0].position = joint_position_command_[0];
        SendLowROS.motorCmd[FL_0].velocity = 0;
        SendLowROS.motorCmd[FL_0].positionStiffness = Kp[0];
        SendLowROS.motorCmd[FL_0].velocityStiffness = Kv[0];
        SendLowROS.motorCmd[FL_0].torque = 0.65f;

        SendLowROS.motorCmd[FL_1].position = joint_position_command_[1];
        SendLowROS.motorCmd[FL_1].velocity = 0;
        SendLowROS.motorCmd[FL_1].positionStiffness = Kp[1];
        SendLowROS.motorCmd[FL_1].velocityStiffness = Kv[1];
        SendLowROS.motorCmd[FL_1].torque = 0.0f;

        SendLowROS.motorCmd[FL_2].position =  joint_position_command_[2];
        SendLowROS.motorCmd[FL_2].velocity = 0;
        SendLowROS.motorCmd[FL_2].positionStiffness = Kp[2];
        SendLowROS.motorCmd[FL_2].velocityStiffness = Kv[2];
        SendLowROS.motorCmd[FL_2].torque = 0.0f;

        SendLowROS.motorCmd[RR_0].position = joint_position_command_[6];
        SendLowROS.motorCmd[RR_0].velocity = 0;
        SendLowROS.motorCmd[RR_0].positionStiffness = Kp[0];
        SendLowROS.motorCmd[RR_0].velocityStiffness = Kv[0];
        SendLowROS.motorCmd[RR_0].torque = -0.65f;

        SendLowROS.motorCmd[RR_1].position = joint_position_command_[7];
        SendLowROS.motorCmd[RR_1].velocity = 0;
        SendLowROS.motorCmd[RR_1].positionStiffness = Kp[1];
        SendLowROS.motorCmd[RR_1].velocityStiffness = Kv[1];
        SendLowROS.motorCmd[RR_1].torque = 0.0f;

        SendLowROS.motorCmd[RR_2].position =  joint_position_command_[8];
        SendLowROS.motorCmd[RR_2].velocity = 0;
        SendLowROS.motorCmd[RR_2].positionStiffness = Kp[2];
        SendLowROS.motorCmd[RR_2].velocityStiffness = Kv[2];
        SendLowROS.motorCmd[RR_2].torque = 0.0f;


        SendLowROS.motorCmd[RL_0].position = joint_position_command_[9];
        SendLowROS.motorCmd[RL_0].velocity = 0;
        SendLowROS.motorCmd[RL_0].positionStiffness = Kp[0];
        SendLowROS.motorCmd[RL_0].velocityStiffness = Kv[0];
        SendLowROS.motorCmd[RL_0].torque = 0.65f;

        SendLowROS.motorCmd[RL_1].position = joint_position_command_[10];
        SendLowROS.motorCmd[RL_1].velocity = 0;
        SendLowROS.motorCmd[RL_1].positionStiffness = Kp[1];
        SendLowROS.motorCmd[RL_1].velocityStiffness = Kv[1];
        SendLowROS.motorCmd[RL_1].torque = 0.0f;

        SendLowROS.motorCmd[RL_2].position =  joint_position_command_[11];
        SendLowROS.motorCmd[RL_2].velocity = 0;
        SendLowROS.motorCmd[RL_2].positionStiffness = Kp[2];
        SendLowROS.motorCmd[RL_2].velocityStiffness = Kv[2];
        SendLowROS.motorCmd[RL_2].torque = 0.0f;
    }

////MXR::NOTE: For set joint position
    if( motiontime >= 100&&!Init_flag()&&controller_name=="all_joints_position_group_controller"){
        if( motiontime == 100){
            Kp[0] = 0.232; Kp[1] = 0.032; Kp[2] = 0.032;
            Kv[0] = 0.16; Kv[1] = 0.16; Kv[2] = 0.16;
        }

        if( motiontime == 500){
            Kp[0] = 0.52; Kp[1] = 0.062; Kp[2] = 0.062;
            Kv[0] = 0.12; Kv[1] = 0.12; Kv[2] = 0.12;
        }

        if( motiontime == 800){
            Kp[0] = 0.92; Kp[1] = 0.092; Kp[2] = 0.092;
            Kv[0] = 0.06; Kv[1] = 0.06; Kv[2] = 0.06;
        }

        if( motiontime == 1300){
            Kp[0] = 1.32; Kp[1] = 0.132; Kp[2] = 0.132;
            Kv[0] = 0.02; Kv[1] = 0.02; Kv[2] = 0.02;
        }

        if( motiontime == 1600){
            Kp[0] = 1.84; Kp[1] = 0.363; Kp[2] = 0.363;
            Kv[0] = 0.02; Kv[1] = 0.02; Kv[2] = 0.02;
        }
        if( motiontime == 2000){
            Kp[0] = 2.34; Kp[1] = 0.693; Kp[2] = 0.693;
            Kv[0] = 0.02; Kv[1] = 0.02; Kv[2] = 0.02;
        }

        SendLowROS.motorCmd[FR_0].position = joint_position_command_[3];
        SendLowROS.motorCmd[FR_0].velocity = 0;
        SendLowROS.motorCmd[FR_0].positionStiffness = Kp[0];
        SendLowROS.motorCmd[FR_0].velocityStiffness = Kv[0];
        SendLowROS.motorCmd[FR_0].torque = -0.65f;

        SendLowROS.motorCmd[FR_1].position = joint_position_command_[4];
        SendLowROS.motorCmd[FR_1].velocity = 0;
        SendLowROS.motorCmd[FR_1].positionStiffness = Kp[1];
        SendLowROS.motorCmd[FR_1].velocityStiffness = Kv[1];
        SendLowROS.motorCmd[FR_1].torque = 0.0f;

        SendLowROS.motorCmd[FR_2].position =  joint_position_command_[5];
        SendLowROS.motorCmd[FR_2].velocity = 0;
        SendLowROS.motorCmd[FR_2].positionStiffness = Kp[2];
        SendLowROS.motorCmd[FR_2].velocityStiffness = Kv[2];
        SendLowROS.motorCmd[FR_2].torque = 0.0f;

        SendLowROS.motorCmd[FL_0].position = joint_position_command_[0];
        SendLowROS.motorCmd[FL_0].velocity = 0;
        SendLowROS.motorCmd[FL_0].positionStiffness = Kp[0];
        SendLowROS.motorCmd[FL_0].velocityStiffness = Kv[0];
        SendLowROS.motorCmd[FL_0].torque = 0.65f;

        SendLowROS.motorCmd[FL_1].position = joint_position_command_[1];
        SendLowROS.motorCmd[FL_1].velocity = 0;
        SendLowROS.motorCmd[FL_1].positionStiffness = Kp[1];
        SendLowROS.motorCmd[FL_1].velocityStiffness = Kv[1];
        SendLowROS.motorCmd[FL_1].torque = 0.0f;

        SendLowROS.motorCmd[FL_2].position =  joint_position_command_[2];
        SendLowROS.motorCmd[FL_2].velocity = 0;
        SendLowROS.motorCmd[FL_2].positionStiffness = Kp[2];
        SendLowROS.motorCmd[FL_2].velocityStiffness = Kv[2];
        SendLowROS.motorCmd[FL_2].torque = 0.0f;

        SendLowROS.motorCmd[RR_0].position = joint_position_command_[6];
        SendLowROS.motorCmd[RR_0].velocity = 0;
        SendLowROS.motorCmd[RR_0].positionStiffness = Kp[0];
        SendLowROS.motorCmd[RR_0].velocityStiffness = Kv[0];
        SendLowROS.motorCmd[RR_0].torque = -0.65f;

        SendLowROS.motorCmd[RR_1].position = joint_position_command_[7];
        SendLowROS.motorCmd[RR_1].velocity = 0;
        SendLowROS.motorCmd[RR_1].positionStiffness = Kp[1];
        SendLowROS.motorCmd[RR_1].velocityStiffness = Kv[1];
        SendLowROS.motorCmd[RR_1].torque = 0.0f;

        SendLowROS.motorCmd[RR_2].position =  joint_position_command_[8];
        SendLowROS.motorCmd[RR_2].velocity = 0;
        SendLowROS.motorCmd[RR_2].positionStiffness = Kp[2];
        SendLowROS.motorCmd[RR_2].velocityStiffness = Kv[2];
        SendLowROS.motorCmd[RR_2].torque = 0.0f;


        SendLowROS.motorCmd[RL_0].position = joint_position_command_[9];
        SendLowROS.motorCmd[RL_0].velocity = 0;
        SendLowROS.motorCmd[RL_0].positionStiffness = Kp[0];
        SendLowROS.motorCmd[RL_0].velocityStiffness = Kv[0];
        SendLowROS.motorCmd[RL_0].torque = 0.65f;

        SendLowROS.motorCmd[RL_1].position = joint_position_command_[10];
        SendLowROS.motorCmd[RL_1].velocity = 0;
        SendLowROS.motorCmd[RL_1].positionStiffness = Kp[1];
        SendLowROS.motorCmd[RL_1].velocityStiffness = Kv[1];
        SendLowROS.motorCmd[RL_1].torque = 0.0f;

        SendLowROS.motorCmd[RL_2].position =  joint_position_command_[11];
        SendLowROS.motorCmd[RL_2].velocity = 0;
        SendLowROS.motorCmd[RL_2].positionStiffness = Kp[2];
        SendLowROS.motorCmd[RL_2].velocityStiffness = Kv[2];
        SendLowROS.motorCmd[RL_2].torque = 0.0f;
    }
//##############################(torque_control)######################################################
//         SendLowROS.motorCmd[FL_0].position = PosStopF;
//         SendLowROS.motorCmd[FL_0].velocity = VelStopF;
//         SendLowROS.motorCmd[FL_0].positionStiffness = 0;
//         SendLowROS.motorCmd[FL_0].velocityStiffness = 0;
//         SendLowROS.motorCmd[FL_0].torque =torque_set[0]+0.65f;

//         SendLowROS.motorCmd[FL_1].position = PosStopF;
//         SendLowROS.motorCmd[FL_1].velocity = VelStopF;
//         SendLowROS.motorCmd[FL_1].positionStiffness = 0;
//         SendLowROS.motorCmd[FL_1].velocityStiffness = 0;
//         SendLowROS.motorCmd[FL_1].torque =torque_set[1];

//         SendLowROS.motorCmd[FL_2].position = PosStopF;
//         SendLowROS.motorCmd[FL_2].velocity = VelStopF;
//         SendLowROS.motorCmd[FL_2].positionStiffness = 0;
//         SendLowROS.motorCmd[FL_2].velocityStiffness = 0;
//         SendLowROS.motorCmd[FL_2].torque =torque_set[2];
    if(!Init_flag()&&controller_name=="base_balance_controller"){


//                if(joint_effort_command_[3]>5.0){
//                    joint_effort_command_[3]=5.0;
//                }
//                if(joint_effort_command_[3]<-5.0){
//                    joint_effort_command_[3]=-5.0;
//                }

//                 SendLowROS.motorCmd[FR_0].position = PosStopF;
//                 SendLowROS.motorCmd[FR_0].velocity = VelStopF;
//                 SendLowROS.motorCmd[FR_0].positionStiffness = 0;
//                 SendLowROS.motorCmd[FR_0].velocityStiffness = 0;
//                 SendLowROS.motorCmd[FR_0].torque =joint_effort_command_[3];

//        if(joint_effort_command_[4]>5.0){
//            joint_effort_command_[4]=5.0;
//        }
//        if(joint_effort_command_[4]<-5.0){
//            joint_effort_command_[4]=-5.0;
//        }

//         SendLowROS.motorCmd[FR_1].position = PosStopF;
//         SendLowROS.motorCmd[FR_1].velocity = VelStopF;
//         SendLowROS.motorCmd[FR_1].positionStiffness = 0;
//         SendLowROS.motorCmd[FR_1].velocityStiffness = 0;
//         SendLowROS.motorCmd[FR_1].torque =joint_effort_command_[4];


//        if(joint_effort_command_[5]>5.0){
//            joint_effort_command_[5]=5.0;
//        }
//        if(joint_effort_command_[5]<-5.0){
//            joint_effort_command_[5]=-5.0;
//        }

//         SendLowROS.motorCmd[FR_2].position = PosStopF;
//         SendLowROS.motorCmd[FR_2].velocity = VelStopF;
//         SendLowROS.motorCmd[FR_2].positionStiffness = 0;
//         SendLowROS.motorCmd[FR_2].velocityStiffness = 0;
//         SendLowROS.motorCmd[FR_2].torque =joint_effort_command_[5];

//        if(joint_effort_command_[0]>5.0){
//            joint_effort_command_[0]=5.0;
//        }
//        if(joint_effort_command_[0]<-5.0){
//            joint_effort_command_[0]=-5.0;
//        }

//         SendLowROS.motorCmd[FL_0].position = PosStopF;
//         SendLowROS.motorCmd[FL_0].velocity = VelStopF;
//         SendLowROS.motorCmd[FL_0].positionStiffness = 0;
//         SendLowROS.motorCmd[FL_0].velocityStiffness = 0;
//         SendLowROS.motorCmd[FL_0].torque =joint_effort_command_[0];

//                 if(joint_effort_command_[1]>5.0){
//                     joint_effort_command_[1]=5.0;
//                 }
//                 if(joint_effort_command_[1]<-5.0){
//                     joint_effort_command_[1]=-5.0;
//                 }

//                  SendLowROS.motorCmd[FL_1].position = PosStopF;
//                  SendLowROS.motorCmd[FL_1].velocity = VelStopF;
//                  SendLowROS.motorCmd[FL_1].positionStiffness = 0;
//                  SendLowROS.motorCmd[FL_1].velocityStiffness = 0;
//                  SendLowROS.motorCmd[FL_1].torque =joint_effort_command_[1];
//         if(joint_effort_command_[2]>5.0){
//             joint_effort_command_[2]=5.0;
//         }
//         if(joint_effort_command_[2]<-5.0){
//             joint_effort_command_[2]=-5.0;
//         }

//          SendLowROS.motorCmd[FL_2].position = PosStopF;
//          SendLowROS.motorCmd[FL_2].velocity = VelStopF;
//          SendLowROS.motorCmd[FL_2].positionStiffness = 0;
//          SendLowROS.motorCmd[FL_2].velocityStiffness = 0;
//          SendLowROS.motorCmd[FL_2].torque =joint_effort_command_[2];

//        if(joint_effort_command_[11]>5.0){
//            joint_effort_command_[11]=5.0;
//        }
//        if(joint_effort_command_[11]<-5.0){
//            joint_effort_command_[11]=-5.0;
//        }

//         SendLowROS.motorCmd[RL_2].position = PosStopF;
//         SendLowROS.motorCmd[RL_2].velocity = VelStopF;
//         SendLowROS.motorCmd[RL_2].positionStiffness = 0;
//         SendLowROS.motorCmd[RL_2].velocityStiffness = 0;
//         SendLowROS.motorCmd[RL_2].torque =joint_effort_command_[11];

//        if(joint_effort_command_[8]>5.0){
//            joint_effort_command_[8]=5.0;
//        }
//        if(joint_effort_command_[8]<-5.0){
//            joint_effort_command_[8]=-5.0;
//        }

//         SendLowROS.motorCmd[RR_2].position = PosStopF;
//         SendLowROS.motorCmd[RR_2].velocity = VelStopF;
//         SendLowROS.motorCmd[RR_2].positionStiffness = 0;
//         SendLowROS.motorCmd[RR_2].velocityStiffness = 0;
//         SendLowROS.motorCmd[RR_2].torque =joint_effort_command_[8];

//         SendLowROS.motorCmd[RR_0].position = PosStopF;
//         SendLowROS.motorCmd[RR_0].velocity = VelStopF;
//         SendLowROS.motorCmd[RR_0].positionStiffness = 0;
//         SendLowROS.motorCmd[RR_0].velocityStiffness = 0;
//         SendLowROS.motorCmd[RR_0].torque =torque_set[6]-0.65f;

//         SendLowROS.motorCmd[RR_1].position = PosStopF;
//         SendLowROS.motorCmd[RR_1].velocity = VelStopF;
//         SendLowROS.motorCmd[RR_1].positionStiffness = 0;
//         SendLowROS.motorCmd[RR_1].velocityStiffness = 0;
//         SendLowROS.motorCmd[RR_1].torque =torque_set[7];

//         SendLowROS.motorCmd[RR_2].position = PosStopF;
//         SendLowROS.motorCmd[RR_2].velocity = VelStopF;
//         SendLowROS.motorCmd[RR_2].positionStiffness = 0;
//         SendLowROS.motorCmd[RR_2].velocityStiffness = 0;
//         SendLowROS.motorCmd[RR_2].torque =torque_set[8];

//         SendLowROS.motorCmd[RL_0].position = PosStopF;
//         SendLowROS.motorCmd[RL_0].velocity = VelStopF;
//         SendLowROS.motorCmd[RL_0].positionStiffness = 0;
//         SendLowROS.motorCmd[RL_0].velocityStiffness = 0;
//         SendLowROS.motorCmd[RL_0].torque =torque_set[9]+0.65f;

//         SendLowROS.motorCmd[RL_1].position = PosStopF;
//         SendLowROS.motorCmd[RL_1].velocity = VelStopF;
//         SendLowROS.motorCmd[RL_1].positionStiffness = 0;
//         SendLowROS.motorCmd[RL_1].velocityStiffness = 0;
//         SendLowROS.motorCmd[RL_1].torque =torque_set[10];

//         SendLowROS.motorCmd[RL_2].position = PosStopF;
//         SendLowROS.motorCmd[RL_2].velocity = VelStopF;
//         SendLowROS.motorCmd[RL_2].positionStiffness = 0;
//         SendLowROS.motorCmd[RL_2].velocityStiffness = 0;
//         SendLowROS.motorCmd[RL_2].torque =torque_set[11];
}
    // gravity compensation
//    SendLowROS.motorCmd[FR_0].torque = -0.65f;
//    SendLowROS.motorCmd[FL_0].torque = +0.65f;
//    SendLowROS.motorCmd[RR_0].torque = -0.65f;
//    SendLowROS.motorCmd[RL_0].torque = +0.65f;



    multi_servoCmd.clear();
    memcpy(&SendLowLCM, &SendLowROS, sizeof(LowCmd));
    roslcm.Send(SendLowLCM);

    lock.unlock();

    //boost::recursive_mutex::scoped_lock lock(r_mutex_);

}

void RobotStateLcmHardwareInterface::registerJointLimits(const std::string& joint_name,
                         const hardware_interface::JointHandle& joint_handle,
                         const ControlMethod ctrl_method,
                         const ros::NodeHandle& joint_limit_nh,
                         const urdf::Model *const urdf_model,
                         int *const joint_type, double *const lower_limit,
                         double *const upper_limit, double *const effort_limit)
{
  *joint_type = urdf::Joint::UNKNOWN;
  *lower_limit = -std::numeric_limits<double>::max();
  *upper_limit = std::numeric_limits<double>::max();
  *effort_limit = std::numeric_limits<double>::max();

  joint_limits_interface::JointLimits limits;
  bool has_limits = false;
  joint_limits_interface::SoftJointLimits soft_limits;
  bool has_soft_limits = false;

  if (urdf_model != NULL)
  {
    const urdf::JointConstSharedPtr urdf_joint = urdf_model->getJoint(joint_name);
    if (urdf_joint != NULL)
    {
      *joint_type = urdf_joint->type;
      // Get limits from the URDF file.
      if (joint_limits_interface::getJointLimits(urdf_joint, limits))
        has_limits = true;
      if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
        has_soft_limits = true;
    }
  }
  // Get limits from the parameter server.
  if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
    has_limits = true;

  if (!has_limits)
    return;

  if (*joint_type == urdf::Joint::UNKNOWN)
  {
    // Infer the joint type.

    if (limits.has_position_limits)
    {
      *joint_type = urdf::Joint::REVOLUTE;
    }
    else
    {
      if (limits.angle_wraparound)
        *joint_type = urdf::Joint::CONTINUOUS;
      else
        *joint_type = urdf::Joint::PRISMATIC;
    }
  }

  if (limits.has_position_limits)
  {
    *lower_limit = limits.min_position;
    *upper_limit = limits.max_position;
  }
  if (limits.has_effort_limits)
    *effort_limit = limits.max_effort;

  if (has_soft_limits)
  {
    switch (ctrl_method)
    {
      case EFFORT:
        {
          const joint_limits_interface::EffortJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          ej_limits_interface_.registerHandle(limits_handle);
        }
        break;
      case POSITION:
        {
          const joint_limits_interface::PositionJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          pj_limits_interface_.registerHandle(limits_handle);
        }
        break;
      case VELOCITY:
        {
          const joint_limits_interface::VelocityJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          vj_limits_interface_.registerHandle(limits_handle);
        }
        break;
    }
  }
  else
  {
    switch (ctrl_method)
    {
      case EFFORT:
        {
          const joint_limits_interface::EffortJointSaturationHandle
            sat_handle(joint_handle, limits);
          ej_sat_interface_.registerHandle(sat_handle);
        }
        break;
      case POSITION:
        {
          const joint_limits_interface::PositionJointSaturationHandle
            sat_handle(joint_handle, limits);
          pj_sat_interface_.registerHandle(sat_handle);
        }
        break;
      case VELOCITY:
        {
          const joint_limits_interface::VelocityJointSaturationHandle
            sat_handle(joint_handle, limits);
          vj_sat_interface_.registerHandle(sat_handle);
        }
        break;
    }
  }
}


}

PLUGINLIB_EXPORT_CLASS(laikago_ros_control::RobotStateLcmHardwareInterface, hardware_interface::RobotHW)
