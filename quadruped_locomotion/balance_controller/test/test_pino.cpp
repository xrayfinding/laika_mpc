#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include <iostream>
#include "urdf/model.h"
#include "urdf_parser/urdf_parser.h"
#include "laikago/rcg/transforms.h"
#include "laikago/rcg/declarations.h"
#include "laikago/rcg/inertia_properties.h"
#include "laikago/rcg/inverse_dynamics.h"
#include "pronto_laikago_commons/feet_contact_forces.hpp"
#include "pronto_laikago_commons/feet_jacobians.hpp"
#include "pronto_laikago_commons/forward_kinematics.hpp"
#include "free_gait_msgs/RobotState.h"
#include "free_gait_core/TypeDefs.hpp"
#include "sensor_msgs/Imu.h"
#include "boost/thread.hpp"
#include "boost/bind.hpp"
// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "/home/glx/depends/pinocchio/models"
#endif
//GLX:Attention the leg order in robcogen is lf rf lh rh ,our leg order is lf rf rh lh
laikago::FeetContactForces feet_forces;
laikago::FeetJacobians feet_jacs;
laikago::ForwardKinematics fwd_kin;
laikago::FeetContactForces::JointState q;
laikago::FeetContactForces::JointState qd;
laikago::FeetContactForces::JointState tau;
laikago::FeetContactForces::JointState qdd;
Eigen::Quaterniond orient;
laikago::FeetContactForces::LegID leg;
laikago::FeetContactForces::Vector3d xd;
laikago::FeetContactForces::Vector3d xdd;
laikago::FeetContactForces::Vector3d omega;
laikago::FeetContactForces::Vector3d omegad;
laikago::FeetContactForces::LegVectorMap feet_force_comute;

void basePoseCallback(const free_gait_msgs::RobotStateConstPtr& robot_state){
    geometry_msgs::PoseWithCovariance base_pose_in_world_;
    base_pose_in_world_.pose = robot_state->base_pose.pose.pose;

    Pose base_pose = Pose(Position(base_pose_in_world_.pose.position.x,
                          base_pose_in_world_.pose.position.y,
                          base_pose_in_world_.pose.position.z),
                 RotationQuaternion(base_pose_in_world_.pose.orientation.w,
                                    base_pose_in_world_.pose.orientation.x,
                                    base_pose_in_world_.pose.orientation.y,
                                    base_pose_in_world_.pose.orientation.z));
    orient.w() = base_pose_in_world_.pose.orientation.w;
    orient.x() = base_pose_in_world_.pose.orientation.x;
    orient.y() = base_pose_in_world_.pose.orientation.y;
    orient.z() = base_pose_in_world_.pose.orientation.z;
    q.setZero();
    qd.setZero();
    qdd.setZero();
    std::vector<double> joint_ang = robot_state->lf_leg_joints.position;
    std::vector<double> joint_vel = robot_state->lf_leg_joints.velocity;
    std::vector<double> joint_tau = robot_state->lf_leg_joints.effort;
    q(0) = joint_ang[0];
    q(1) = joint_ang[1];
    q(2) = joint_ang[2];
    qd(0) = joint_vel[0];
    qd(1) = joint_vel[1];
    qd(2) = joint_vel[2];
    tau(0) = joint_tau[0];
    tau(1) = joint_tau[1];
    tau(2) = joint_tau[2];
    joint_ang = robot_state->rf_leg_joints.position;
    joint_vel = robot_state->rf_leg_joints.velocity;
    joint_tau = robot_state->rf_leg_joints.effort;
    q(3) = joint_ang[0];
    q(4) = joint_ang[1];
    q(5) = joint_ang[2];
    qd(3) = joint_vel[0];
    qd(4) = joint_vel[1];
    qd(5) = joint_vel[2];
    tau(3) = joint_tau[0];
    tau(4) = joint_tau[1];
    tau(5) = joint_tau[2];
    joint_ang = robot_state->lh_leg_joints.position;
    joint_vel = robot_state->lh_leg_joints.velocity;
    joint_tau = robot_state->lh_leg_joints.effort;
    q(6) = joint_ang[0];
    q(7) = joint_ang[1];
    q(8) = joint_ang[2];
    qd(6) = joint_vel[0];
    qd(7) = joint_vel[1];
    qd(8) = joint_vel[2];
    tau(6) = joint_tau[0];
    tau(7) = joint_tau[1];
    tau(8) = joint_tau[2];
    joint_ang = robot_state->rh_leg_joints.position;
    joint_vel = robot_state->rh_leg_joints.velocity;
    joint_tau = robot_state->rh_leg_joints.effort;
    q(9) = joint_ang[0];
    q(10) = joint_ang[1];
    q(11) = joint_ang[2];
    qd(9) = joint_vel[0];
    qd(10) = joint_vel[1];
    qd(11) = joint_vel[2];
    tau(9) = joint_tau[0];
    tau(10) = joint_tau[1];
    tau(11) = joint_tau[2];
    xd(0) = robot_state->base_pose.twist.twist.linear.x;
    xd(1) = robot_state->base_pose.twist.twist.linear.y;
    xd(2) = robot_state->base_pose.twist.twist.linear.z;
    omega(0) = robot_state->base_pose.twist.twist.angular.x;
    omega(1) = robot_state->base_pose.twist.twist.angular.y;
    omega(2) = robot_state->base_pose.twist.twist.angular.z;
    if(feet_forces.getFeetGRF(q,qd,tau,orient,feet_force_comute,qdd,xd,xdd,omega,omegad)){
        std::cout << feet_force_comute[laikago::FeetContactForces::LegID::RH](0)<<" "
                  << feet_force_comute[laikago::FeetContactForces::LegID::RH](1)<<" "
                  << feet_force_comute[laikago::FeetContactForces::LegID::RH](2)<<" "
                  << std::endl;
    }
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg){
    xdd(0) = imu_msg->linear_acceleration.x;
    xdd(1) = imu_msg->linear_acceleration.y;
    xdd(2) = imu_msg->linear_acceleration.z;
}
int main(int argc, char ** argv)
{
//    using namespace pinocchio;
//    //You should change here to set up your own URDF file or just pass it as an argument of this example.
//    std::string urdf_filename = (argc<=1) ? PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/anymal_b_simple_description/robots/anymal.urdf") : argv[1];
//    pinocchio::Model model;
//    const bool a1s = true;
//    pinocchio::urdf::buildModel(urdf_filename,model,a1s);
//    GeometryModel collision_model;
//    std::string mesh_dir = PINOCCHIO_MODEL_DIR;
//    pinocchio::urdf::buildGeom(model,urdf_filename,COLLISION,collision_model,mesh_dir);
//    GeometryModel visual_model;
//    pinocchio::urdf::buildGeom(model,urdf_filename,VISUAL,visual_model,mesh_dir);
//    std::cout << "model name: " << model.name << std::endl;

//    // Create data required by the algorithms
//    Data data(model);
//    GeometryData collision_data(collision_model);
//    GeometryData visual_data(visual_model);

//    // Sample a random configuration
//    Eigen::VectorXd q = randomConfiguration(model);
//    std::cout << "q: " << q.transpose() << std::endl;
//    q.setZero();
//    // Perform the forward kinematics over the kinematic tree
//    forwardKinematics(model,data,q);
//    updateGeometryPlacements(model,data,collision_model,collision_data);
//    updateGeometryPlacements(model,data,visual_model,visual_data);

//    // Print out the placement of each joint of the kinematic tree
//    for(JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
//    std::cout << std::setw(24) << std::left
//              << model.names[joint_id] << ": "
//              << std::fixed << std::setprecision(2)
//              << data.oMi[joint_id].translation().transpose()
//              << std::endl;
//    for(GeomIndex geom_id=0;geom_id<(GeomIndex)collision_model.ngeoms;++geom_id){
//        std::cout << geom_id << ": "
//                  << std::fixed << std::setprecision(2)
//                  << collision_data.oMg[geom_id].translation().transpose()
//                  << std::endl;
//    }
//SE2:Present the 2D translation and rotation

    //  SpecialEuclideanOperationTpl<2,double,0> aSE2;
    //  SpecialEuclideanOperationTpl<2,double,0>::ConfigVector_t pose1,pose2;
    //  SpecialEuclideanOperationTpl<2,double,0>::TangentVector_t delta_1;
    //  pose1(0)=1.0;
    //  pose1(1)=1.0;
    //  pose1(2)=cos(M_PI/4.0);
    //  pose1(3)=sin(M_PI/4.0);
    //  pose2(0)=3.0;
    //  pose2(1)=-1.0;
    //  pose2(2)=cos(M_PI/4.0);
    //  pose2(3)=sin(M_PI/4.0);
    //  aSE2.difference(pose1,pose2,delta_1);
    //  SpecialEuclideanOperationTpl<2,double,0>::ConfigVector_t pose3;
    //  aSE2.integrate(pose1,delta_1,pose3);
    //  std::cout<<delta_1<<std::endl;
    //  std::cout<<pose3<<std::endl;

//SE3:present the 3D

    //  SpecialEuclideanOperationTpl<3,double,0>::ConfigVector_t pose3_s,pose3_e,pose3_t,pose3_t1;
    //  SpecialEuclideanOperationTpl<3,double,0>::TangentVector_t delta3;
    //  SpecialEuclideanOperationTpl<3,double,0> ase3;
    //  pose3_s(0) = 1.0; pose3_s(1) = 1.0;
    //  pose3_s(2) = 1 ; pose3_s(3) = -0.13795 ;
    //  pose3_s(4) = 0.13795; pose3_s(5) = 0.69352; pose3_s(6) = 0.69352;
    //  pose3_e(0) = 4; pose3_e(1) = 3 ;
    //  pose3_e(2) = 3 ; pose3_e(3) = -0.13795;
    //  pose3_e(4) = 0.13795; pose3_e(5) = 0.69352; pose3_e(6) = 0.69352;
    //  ase3.difference(pose3_s,pose3_e,delta3);
    //  ase3.integrate(pose3_s,delta3,pose3_t1);
    //  std::cout << pose3_t1 << std::endl;
    //  ase3.interpolate(pose3_s,pose3_e,0.5f,pose3_t1);//interpolation to plot a trajectory
    //  std::cout << pose3_t1 << std::endl;
    laikago::rcg::JointState qlaika,qdlaika,qddlaika,taulaika;
    laikago::rcg::InertiaProperties inertia_prop_;
    laikago::rcg::MotionTransforms motion_transf_;
    laikago::rcg::InverseDynamics inverse_dynamics_(inertia_prop_, motion_transf_);
    boost::recursive_mutex lock1;
    ros::init(argc,argv,"pronto_test_foot_force");
    ros::NodeHandle nh;
    ros::Subscriber base_sub = nh.subscribe("/gazebo/robot_states", 1 , basePoseCallback);
    ros::Subscriber imu_sub = nh.subscribe("/imu",1,imuCallback);
    ros::Rate rate(400);
    omegad.setZero();
    xdd.setZero();
    while(ros::ok()){
        ros::spinOnce();
        //std::cout << q << qd << tau << orient << qdd << xd << xdd << omega << omegad << std::endl;

        rate.sleep();
    }
}

