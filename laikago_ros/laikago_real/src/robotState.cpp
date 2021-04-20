/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <laikago_msgs/LowCmd.h>
#include <laikago_msgs/LowState.h>
#include "laikago_sdk/laikago_sdk.hpp"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"

using namespace laikago;

sensor_msgs::JointState all_joint_state;
sensor_msgs::Imu imu_msg;
geometry_msgs::WrenchStamped lf_contact_force,rf_contact_force,rh_contact_force,lh_contact_force;

LowCmd SendLowLCM = {0};
LowState RecvLowLCM = {0};
laikago_msgs::LowCmd SendLowROS;
laikago_msgs::LowState RecvLowROS;

Control control(LOWLEVEL);
LCM roslcm;
boost::mutex mutex;
ros::Publisher all_joint_pub,imu_pub,lf_foot_contact_force_pub,rf_foot_contact_force_pub,
rh_foot_contact_force_pub,lh_foot_contact_force_pub;

void* update_loop(void* data)
{
    while(ros::ok){
        boost::mutex::scoped_lock lock(mutex);
        roslcm.Recv();
        lock.unlock();
        usleep(2000);
    }
}

int main(int argc, char *argv[])
{
//    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
//              << "Make sure the robot is hung up." << std::endl
//              << "Press Enter to continue..." << std::endl;
//    std::cin.ignore();

    ros::init(argc, argv, "robotState");
    ros::NodeHandle n;
    ros::Rate loop_rate(500);
    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop, NULL);

//    SendLowROS.levelFlag = LOWLEVEL;
//    for(int i = 1; i<13; i++){
//        SendLowROS.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
//    }

    all_joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states",1);
    imu_pub = n.advertise<sensor_msgs::Imu>("/imu",1);
    lf_foot_contact_force_pub = n.advertise<geometry_msgs::WrenchStamped>("/lf_contact_force",1);
    rf_foot_contact_force_pub = n.advertise<geometry_msgs::WrenchStamped>("/rf_contact_force",1);
    rh_foot_contact_force_pub = n.advertise<geometry_msgs::WrenchStamped>("/rh_contact_force",1);
    lh_foot_contact_force_pub = n.advertise<geometry_msgs::WrenchStamped>("/lh_contact_force",1);

    while (ros::ok()){
        //motiontime++;
        roslcm.Get(RecvLowLCM);
        memcpy(&RecvLowROS, &RecvLowLCM, sizeof(LowState));
        all_joint_state.header.stamp = ros::Time::now();
        all_joint_state.name = {"LF_HAA","LF_HFE","LF_KFE",
                                "RF_HAA","RF_HFE","RF_KFE",
                                "RH_HAA","RH_HFE","RH_KFE",
                                "LH_HAA","LH_HFE","LH_KFE"};
        //MXR::NOTE: must resize this container
        all_joint_state.position.resize(12);
        all_joint_state.velocity.resize(12);
        all_joint_state.effort.resize(12);

        all_joint_state.position[0] =static_cast<double>(RecvLowROS.motorState[FL_0].position);
        all_joint_state.position[1] =static_cast<double>(RecvLowROS.motorState[FL_1].position);
        all_joint_state.position[2] =static_cast<double>(RecvLowROS.motorState[FL_2].position);
        all_joint_state.position[3] =static_cast<double>(RecvLowROS.motorState[FR_0].position);
        all_joint_state.position[4] =static_cast<double>(RecvLowROS.motorState[FR_1].position);
        all_joint_state.position[5] =static_cast<double>(RecvLowROS.motorState[FR_2].position);
        all_joint_state.position[6] =static_cast<double>(RecvLowROS.motorState[RR_0].position);
        all_joint_state.position[7] =static_cast<double>(RecvLowROS.motorState[RR_1].position);
        all_joint_state.position[8] =static_cast<double>(RecvLowROS.motorState[RR_2].position);
        all_joint_state.position[9] =static_cast<double>(RecvLowROS.motorState[RL_0].position);
        all_joint_state.position[10] =static_cast<double>(RecvLowROS.motorState[RL_1].position);
        all_joint_state.position[11] =static_cast<double>(RecvLowROS.motorState[RL_2].position);

        all_joint_state.velocity[0] =static_cast<double>(RecvLowROS.motorState[FL_0].velocity);
        all_joint_state.velocity[1] =static_cast<double>(RecvLowROS.motorState[FL_1].velocity);
        all_joint_state.velocity[2] =static_cast<double>(RecvLowROS.motorState[FL_2].velocity);
        all_joint_state.velocity[3] =static_cast<double>(RecvLowROS.motorState[FR_0].velocity);
        all_joint_state.velocity[4] =static_cast<double>(RecvLowROS.motorState[FR_1].velocity);
        all_joint_state.velocity[5] =static_cast<double>(RecvLowROS.motorState[FR_2].velocity);
        all_joint_state.velocity[6] =static_cast<double>(RecvLowROS.motorState[RR_0].velocity);
        all_joint_state.velocity[7] =static_cast<double>(RecvLowROS.motorState[RR_1].velocity);
        all_joint_state.velocity[8] =static_cast<double>(RecvLowROS.motorState[RR_2].velocity);
        all_joint_state.velocity[9] =static_cast<double>(RecvLowROS.motorState[RL_0].velocity);
        all_joint_state.velocity[10] =static_cast<double>(RecvLowROS.motorState[RL_1].velocity);
        all_joint_state.velocity[11] =static_cast<double>(RecvLowROS.motorState[RL_2].velocity);

        all_joint_state.effort[0] =static_cast<double>(RecvLowROS.motorState[FL_0].torque);
        all_joint_state.effort[1] =static_cast<double>(RecvLowROS.motorState[FL_1].torque);
        all_joint_state.effort[2] =static_cast<double>(RecvLowROS.motorState[FL_2].torque);
        all_joint_state.effort[3] =static_cast<double>(RecvLowROS.motorState[FR_0].torque);
        all_joint_state.effort[4] =static_cast<double>(RecvLowROS.motorState[FR_1].torque);
        all_joint_state.effort[5] =static_cast<double>(RecvLowROS.motorState[FR_2].torque);
        all_joint_state.effort[6] =static_cast<double>(RecvLowROS.motorState[RR_0].torque);
        all_joint_state.effort[7] =static_cast<double>(RecvLowROS.motorState[RR_1].torque);
        all_joint_state.effort[8] =static_cast<double>(RecvLowROS.motorState[RR_2].torque);
        all_joint_state.effort[9] =static_cast<double>(RecvLowROS.motorState[RL_0].torque);
        all_joint_state.effort[10] =static_cast<double>(RecvLowROS.motorState[RL_1].torque);
        all_joint_state.effort[11] =static_cast<double>(RecvLowROS.motorState[RL_2].torque);

        all_joint_pub.publish(all_joint_state);



        imu_msg.header.stamp = ros::Time::now();
        imu_msg.orientation.w = RecvLowROS.imu.quaternion[0];
        imu_msg.orientation.x = RecvLowROS.imu.quaternion[1];
        imu_msg.orientation.y = RecvLowROS.imu.quaternion[2];
        imu_msg.orientation.z = RecvLowROS.imu.quaternion[3];
        imu_msg.angular_velocity.x = RecvLowROS.imu.gyroscope[0];
        imu_msg.angular_velocity.y = RecvLowROS.imu.gyroscope[1];
        imu_msg.angular_velocity.z = RecvLowROS.imu.gyroscope[2];
        imu_msg.linear_acceleration.x = RecvLowROS.imu.acceleration[0];
        imu_msg.linear_acceleration.y = RecvLowROS.imu.acceleration[1];
        imu_msg.linear_acceleration.z = RecvLowROS.imu.acceleration[2];
        imu_pub.publish(imu_msg);

        //MXR::note: The contact_force isn't the real force in z axis,need to be resolved;
        rf_contact_force.wrench.force.z = RecvLowROS.footForce[0];
        lf_contact_force.wrench.force.z = RecvLowROS.footForce[1];
        rh_contact_force.wrench.force.z = RecvLowROS.footForce[2];
        lh_contact_force.wrench.force.z = RecvLowROS.footForce[3];

        lf_foot_contact_force_pub.publish(lf_contact_force);
        rf_foot_contact_force_pub.publish(rf_contact_force);
        rh_foot_contact_force_pub.publish(rh_contact_force);
        lh_foot_contact_force_pub.publish(lh_contact_force);


        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
