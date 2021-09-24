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
#include "laikago/rcg/forward_dynamics.h"
#include "pronto_laikago_commons/feet_contact_forces.hpp"
#include "pronto_laikago_commons/feet_jacobians.hpp"
#include "pronto_laikago_commons/forward_kinematics.hpp"
#include "free_gait_msgs/RobotState.h"
#include "free_gait_core/TypeDefs.hpp"
#include "sensor_msgs/Imu.h"
#include "boost/thread.hpp"
#include "boost/bind.hpp"
#include "sensor_msgs/JointState.h"

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SparseCore>
#include <assert.h>
#include <vector>
#include <assert.h>
#include "Eigen/Core"
#include "Eigen/SparseCore"
#include "osqp/ctrlc.h"
#include "osqp/osqp.h"
#include "unsupported/Eigen/MatrixFunctions"
#include "pronto_laikago_commons/feet_contact_forces.hpp"
#include "pronto_quadruped_commons/rbd/rbd.h"
#include "pronto_quadruped_commons/geometry/rotations.h"
#include <chrono>
#include "rbdl/rbdl.h"
#include "model/RobotModel.hpp"
#include "model/DynModel.hpp"
#include "model/KinModel.hpp"
#include "model/RobotDefinition.hpp"
#include "utils/pseudo_inverse.hpp"
// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "/home/glx/depends/pinocchio/models"
#endif
using namespace dynacore;
using namespace std;
#define OSQP_INFTY ((c_float)1e30)        // infinity
#define OSQP_NULL 0
#define OSQP_SOLVED (1)
struct modelStateData{
    dynacore::Vector _q;
    dynacore::Vector _qdot;
};

//Test RBDL algorithm
RobotModel* _model = new RobotModel();
struct modelStateData _state = {
    Eigen::VectorXd::Zero(robot::num_q),
    Eigen::VectorXd::Zero(robot::num_qdot)
};

class TestMCG{
public:
    TestMCG(bool out_once):
        inverse_dynamics(inertia_prop_test, motion_trans_test),
        jism_test(inertia_prop_test, force_trans_test),
        one_out(out_once),
        one_out1(out_once)
    {
        Jacobian_laika.resize(18,18);
        M_laika.resize(18,18);
        Jacobian_laika.setZero();
        M_laika.setOnes();
        std::cout<<"Test M C G initial"<<std::endl;
    }
    Eigen::MatrixXd get_M(const laikago::rcg::JointState& q){
        jism_test(q);
        Eigen::MatrixXd xray = jism_test.update(q);
        Eigen::MatrixXd M_test1 = jism_test.getWholeBodyInertia();
        Eigen::MatrixXd M_test2 = jism_test.getFixedBaseBlock();
        Eigen::MatrixXd M_test3 = jism_test.getF();
        jism_test.computeL();
        jism_test.computeInverse();
        Eigen::MatrixXd M_mat_inv = jism_test.getInverse();
        Eigen::MatrixXd H_laika(18,18);
        //H_laika.setZero();
        H_laika.block<6,6>(0,0) = M_test1;
        H_laika.block<12,12>(6,6) = M_test2;
        H_laika.block<6,12>(0,6) = M_test3;
        H_laika.block<12,6>(6,0) = M_test3.transpose();
        Eigen::MatrixXd H_laika_inv = H_laika.inverse();
        //Eigen::Matrix3d M_leg = jism_test.getFixedBaseBlock().block<3, 3>(0, 0);
        //jism_test.computeL();
        //jism_test.computeInverse();
//        Eigen::MatrixXd M_mat = jism_test.getL();
//        Eigen::MatrixXd real_H = M_mat.transpose()*M_mat;
//        auto start_time = std::chrono::system_clock::now();
//        Eigen::MatrixXd Mat_inv2 = real_H.inverse();
//        auto end_time = std::chrono::system_clock::now();
//        auto inv_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time-start_time);
//        if(one_out){
//            std::cout << "time for inverse EIGEN" << double(inv_time.count())*(std::chrono::microseconds::period::num)
//                                 /(std::chrono::microseconds::period::den) << "s"<< std::endl;
//        }
//        start_time = std::chrono::system_clock::now();
//        Eigen::MatrixXd M_mat_inv = jism_test.getInverse();
//        end_time = std::chrono::system_clock::now();
//        inv_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time-start_time);
//        if(one_out){
//            std::cout << "time for inverse RBDL is " << double(inv_time.count())*(std::chrono::microseconds::period::num)
//                                 /(std::chrono::microseconds::period::den) << "s"<< std::endl;
//        }
        //std::cout << " L "<< M_mat.cols() << "  "<< M_mat.rows() << std::endl;
        //std::cout << "inverse size : " <<M_test.rows()<< " : " << M_test.cols()<<std::endl;
        //std::cout << "whole body inertial" << M_test1.size() << std::endl;
        if(one_out){
            std::cout<< std::endl << "*********** (6,6) LU *********"<<std::endl;
            for (int i = 0;i<M_test1.rows();i++) {
                for (int j = 0;j<M_test1.cols();j++) {
                    std::cout<<M_test1(i,j)<<"-";
                }
                std::cout<<std::endl;
            }
            std::cout<< std::endl << "*********** (12,12) RU *********"<<std::endl;
            for (int i = 0;i<M_test2.rows();i++) {
                for (int j = 0;j<M_test2.cols();j++) {
                    std::cout<<M_test2(i,j)<<"-";
                }
                std::cout<<std::endl;
            }
            std::cout<< std::endl << "***********(6,12)RD*********"<<std::endl;
            for (int i = 0;i<M_test3.rows();i++) {
                for (int j = 0;j<M_test3.cols();j++) {
                    std::cout<<M_test3(i,j)<<"-";
                }
                std::cout<<std::endl;
            }
            one_out=false;
            std::cout<< std::endl << "*********** H *********"<<std::endl;
            for (int i = 0;i<xray.rows();i++) {
                for (int j = 0;j<xray.cols();j++) {
                    std::cout<<xray(i,j)<<"-";
                }
                std::cout<<std::endl;
            }
            std::cout<< std::endl << "*********** real_H *********"<<std::endl;
            std::cout << std::endl << H_laika.rows() << "  " << H_laika.cols() << std::endl;
            for (int i = 0;i<H_laika.rows();i++) {
                for (int j = 0;j<H_laika.cols();j++) {
                    std::cout<<H_laika(i,j)<<"-";
                }
                std::cout<<std::endl;
            }
            std::cout<< std::endl << "*********** real_H_T*********"<<std::endl;
            Eigen::MatrixXd H_T = H_laika.transpose();
            for (int i = 0;i<H_T.rows();i++) {
                for (int j = 0;j<H_T.cols();j++) {
                    std::cout<<H_T(i,j)-H_laika(j,i)<<"-";
                }
                std::cout<<std::endl;
            }
            std::cout << "end of this" << std::endl;
            std::cout<< std::endl << "*********** RBDL inv *********"<<std::endl;
            for (int i = 0;i<M_mat_inv.rows();i++) {
                for (int j = 0;j<M_mat_inv.cols();j++) {
                    std::cout<<M_mat_inv(i,j)<<"-";
                }
                std::cout<<std::endl;
            }
            std::cout << "end of this" << std::endl;
            std::cout<< std::endl << "*********** Eigen inv *********"<<std::endl;
            for (int i = 0;i<H_laika_inv.rows();i++) {
                for (int j = 0;j<H_laika_inv.cols();j++) {
                    std::cout<<H_laika_inv(i,j)<<"-";
                }
                std::cout<<std::endl;
            }
            std::cout << "end of this" << std::endl;
        }
        return H_laika;
    }
    void getCG(const Eigen::Quaterniond& orient,
               const laikago::rcg::JointState& q, const laikago::rcg::JointState& qd,
               const pronto::quadruped::Vector3d& omega, const pronto::quadruped::Vector3d& xd){
        //inverse_dynamics.setJointStatus(q);
        laikago::rcg::Force C_base;
        laikago::rcg::JointState C_joint;
        iit::rbd::VelocityVector base_v = iit::rbd::Vector6D::Zero();
        base_v.segment(iit::rbd::LX,3) = xd;
        base_v.segment(iit::rbd::AX,3) = omega;
        inverse_dynamics.C_terms_fully_actuated(C_base,C_joint,base_v,q,qd);
        laikago::rcg::Force G_base;
        laikago::rcg::JointState G_joint;
        iit::rbd::Vector6D gravity_world = iit::rbd::Vector6D::Zero();
        gravity_world(iit::rbd::LZ) = -iit::rbd::g;
        Eigen::Matrix3d Rot = pronto::commons::quatToRotMat(orient);
        iit::rbd::Vector6D gravity_base = iit::rbd::Vector6D::Zero();
        gravity_base.segment(iit::rbd::LX,3) = Rot * gravity_world.segment(iit::rbd::LX, 3);
        inverse_dynamics.G_terms_fully_actuated(G_base, G_joint, gravity_base, q);
        if(one_out1){
            std::cout << "G in the world frame" << std::endl;
            for (int i = 0;i<G_base.size();i++) {
                std::cout << G_base(i) << "-";
            }
            for(int i = 0; i < G_joint.size(); i++){
                std::cout << G_joint(i) << "-";
            }
            std::cout << "C in the world frame" << std::endl;
            for(int i = 0; i < C_base.size(); i++){
                std::cout << C_base(i) << "-";
            }
            for(int i = 0; i < C_joint.size(); i++){
                std::cout << C_joint(i) <<"-";
            }
            std::cout << std::endl;
            one_out1 = false;
        }
        return;
    }
    void getJac(const laikago::rcg::JointState& q){
        Eigen::Matrix3d foot_jac;
        for(int i=0; i<4; i++){
            foot_jac = feet_jacs.getFootJacobian(q,(pronto::quadruped::LegID)i);
            Jacobian_laika.block<3,3>(6+i*3, 6+i*3) = foot_jac;
        }
        //Jacobian_laika.block<6,6>(0,0) =
    }
    void test_dyn_fun(){

    }
private:
    laikago::rcg::InertiaProperties inertia_prop_test;
    laikago::rcg::MotionTransforms motion_trans_test;
    laikago::rcg::ForceTransforms force_trans_test;
    laikago::rcg::InverseDynamics inverse_dynamics;
    laikago::rcg::JSIM jism_test;
    laikago::FeetJacobians feet_jacs;
    Eigen::MatrixXd M_laika;//joint space inertia matrix, and perhaps not right!!!
    Eigen::VectorXd C_laika;//C+G: which is not decided by qdd, only related to the q and qd
    Eigen::MatrixXd S_fun;//selection matrix
    Eigen::MatrixXd Jacobian_laika;//whole body jocbian ???
public:
    bool one_out;
    bool one_out1;
};

//GLX:Attention the leg order in robcogen is lf rf lh rh ,our leg order is lf rf rh lh
laikago::FeetContactForces feet_forces;
laikago::FeetJacobians feet_jacs;
laikago::ForwardKinematics fwd_kin;
laikago::FeetContactForces::JointState q;
laikago::FeetContactForces::JointState qd;
laikago::FeetContactForces::JointState tau;
laikago::FeetContactForces::JointState qdd;
Eigen::Quaterniond orient;
Eigen::Vector3d base_pos_rbdl;
laikago::FeetContactForces::LegID leg;
laikago::FeetContactForces::Vector3d xd;
laikago::FeetContactForces::Vector3d xdd;
laikago::FeetContactForces::Vector3d omega;
laikago::FeetContactForces::Vector3d omegad;
laikago::FeetContactForces::LegVectorMap feet_force_comute;
boost::recursive_mutex r_mutex;
TestMCG test_mcg(true);
Matrix M_rbdl;
dynacore::Vector Jqdq;
Eigen::MatrixXd M_robcogen;

dynacore::Vector Cori_rbdl, Gravity_rbdl;
dynacore::Matrix S_t;
dynacore::Vector Tau_rbdl;
dynacore::Matrix Jac_rbdl;
dynacore::Vector f_leg18;
dynacore::Vector contact_force;
dynacore::Matrix Jac_base;
dynacore::Matrix Jac_leg;
dynacore::Vector acc_cmd;
dynacore::Vector C_and_G_rbdl;
::OSQPWorkspace* workspace(0);
bool initial_run(true);

Eigen::MatrixXd K_weight;
Eigen::MatrixXd L_weight;
Eigen::MatrixXd P_mat;
Eigen::VectorXd G_vec;
Eigen::VectorXd contact_state;
Eigen::MatrixXd constraint_mat;
Eigen::VectorXd cons_lb;
Eigen::VectorXd cons_ub;
Eigen::VectorXd qdd_f_tau_result;

/*Golaoxu:
 *
 * min      0.5*x'Px+g'x
 * f,t,qdd
 * s.t.
 * 1.All_*q=-b-g;
 * 2.Tmin=<tau<=Tmax
 * 3.fmin=<f<=fmax
 * 4.-u*fz=<fx,y<=u*fz; 0<fz<fmax;
 * when t
 * when change this fun into a class,l_weighs,k_weight,P_mat_ptr are private members; And they are resize into fixed size after class initialization;
*/
void Calculate_Pmat_gvec(const dynacore::Matrix& Jac_1218 ,const dynacore::Vector jdqd, const Eigen::VectorXd& contact_state,
                         const Eigen::VectorXd& acc_cmd, const Eigen::VectorXd& f_mpc,const Eigen::MatrixXd& l_weight,
                         const Eigen::MatrixXd& k_weight, Eigen::MatrixXd* P_mat_ptr, Eigen::VectorXd* G_vec_ptr){
    //For compute P
    Eigen::MatrixXd B_qp;
    B_qp.resize(18,18);
    B_qp.block<6,6>(0,0).setIdentity();
    for(int i = 0; i < 4; i++){
        B_qp.block<3,18>(i*3+6, 0) = Jac_1218.block<3,18>(i*3,0);
    }
    Eigen::MatrixXd& p_mat = *P_mat_ptr;
    p_mat.setZero();
    p_mat.block<18,18>(0,0) = 2*B_qp.transpose()*l_weight*B_qp;
    p_mat.block<12,12>(18,18) = 2*k_weight;
    //Golaoxu: make P_mat.det != 0;
    //p_mat.block<12,12>(30,30) = 0.1*k_weight;
    //For compute g
    Eigen::VectorXd q_tmp(18);
    q_tmp.segment(6,12) = jdqd;
    q_tmp-=acc_cmd;
    Eigen::VectorXd& g_vec = *G_vec_ptr;
    //only the contact legs' mpc force are count here;
    g_vec.setZero();
    g_vec.segment(0,18) = 2*B_qp.transpose()*l_weight*q_tmp;
    for(int i = 0; i < 4; i++){
        //g_vec.segment(18,12) = contact_state[i]*2*k_weight*f_mpc;
        double choosen_coef = contact_state(i);
        g_vec.segment(18+3*i, 3) = k_weight.block<3,3>(0,0)*f_mpc.segment(3*i,3)*choosen_coef*2;
    }
}
/*Golaoxu:
 * Jsim: joint state inertial matrix;
 * C_G: C(coriolis and centrifugal force) and Gravity;
*/
void Calculate_constrain_mat(const Eigen::MatrixXd& Jsim,const Eigen::VectorXd C_G ,const Eigen::MatrixXd& Jac_1218,
                             const Eigen::VectorXd& contact_state, const double mass,double friction_coeff,
                             Eigen::MatrixXd* C_mat, Eigen::VectorXd* low_con, Eigen::VectorXd* high_con){
    //Golaoxu : Attention : the state's order is qdd(18) f(12) tau(12)
    //constrains for dynamic equation:
    //1.the part of C for dynamic equation;18
    Eigen::MatrixXd& Constrians_mat = *C_mat;
    Constrians_mat.block<18,18>(0,0) = Jsim;
    Constrians_mat.block<18,12>(0,18) = -Jac_1218.transpose();
    Constrians_mat.block<12,12>(6,30).setIdentity();
    //low and high constrains for dynamic equation
    low_con->segment(0,18) = -C_G;
    high_con->segment(0,18) = -C_G;
    //2.constrains for tau
    Constrians_mat.block<12,12>(38,30).setIdentity();
    low_con->segment(38,12).setConstant(-30.0);
    high_con->segment(38,12).setConstant(30.0);
    //3.constrians for force;20
    Eigen::VectorXd con_fric_up;
    con_fric_up.resize(5);
    con_fric_up.setConstant(10*mass*9.81*(friction_coeff+1));
    con_fric_up(4) = 10*mass*9.81;
    //here, start row and col perhaps not right;
    for (int i = 0; i < 4; i++) {
        high_con->segment(18+i*5, 5) = con_fric_up*contact_state[i];
        low_con->segment(18+i*5, 5).setZero();
        Constrians_mat.block<5,3>(18+5*i, 18+3*i) << -1,
                0, friction_coeff, 1, 0, friction_coeff, 0, -1, friction_coeff,
                0, 1, friction_coeff, 0, 0, 1;
    }
}

void getTauWbc(const Eigen::MatrixXd* Pmat_ptr, const Eigen::VectorXd* Gvec_ptr,
               const Eigen::MatrixXd* Cmat_ptr, const Eigen::VectorXd* low_con,
               const Eigen::VectorXd* high_con, Eigen::VectorXd& tau_result){
    Eigen::SparseMatrix<double, Eigen::ColMajor, long long> objective_matrix = Pmat_ptr->sparseView();
    Eigen::VectorXd objective_vector = *Gvec_ptr;
    Eigen::SparseMatrix<double, Eigen::ColMajor, long long> constraint_matrix = Cmat_ptr->sparseView();

    int num_variables = constraint_matrix.cols();
    int num_constraints = constraint_matrix.rows();

    ::OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = false;
    settings.warm_start = true;
    settings.polish = true;
    settings.adaptive_rho_interval = 25;
    settings.eps_abs = 1e-3;
    settings.eps_rel = 1e-3;
    assert((*Pmat_ptr).cols() == num_variables);
    assert((*Pmat_ptr).rows() == num_variables);
    assert((*Gvec_ptr).size() == num_variables);
    assert((*low_con).size() == num_constraints);
    assert((*high_con).size() == num_constraints);

    Eigen::VectorXd clipped_lower_bound = low_con->cwiseMax(-OSQP_INFTY);
    Eigen::VectorXd clipped_upper_bound = high_con->cwiseMin(OSQP_INFTY);

    ::OSQPData data;
    data.n = num_variables;
    data.m = num_constraints;
    //problem here:
    Eigen::SparseMatrix<double, Eigen::ColMajor, long long> objective_matrix_upper_triangle =
            objective_matrix.triangularView<Eigen::Upper>();
    //cout << objective_matrix_upper_triangle << "\n";
    //Golaoxu : the out matrix is upper triangle , and why?
    ::csc osqp_objective_matrix = {
        objective_matrix_upper_triangle.outerIndexPtr()[num_variables],
        num_variables,
        num_variables,
        const_cast<long long*>(objective_matrix_upper_triangle.outerIndexPtr()),
        const_cast<long long*>(objective_matrix_upper_triangle.innerIndexPtr()),
        const_cast<double*>(objective_matrix_upper_triangle.valuePtr()),
        -1
    };
    data.P = &osqp_objective_matrix;

    ::csc osqp_constraints_matrix = {
        constraint_matrix.outerIndexPtr()[num_variables],
        num_constraints,
        num_variables,
        const_cast<long long*>(constraint_matrix.outerIndexPtr()),
        const_cast<long long*>(constraint_matrix.innerIndexPtr()),
        const_cast<double *>(constraint_matrix.valuePtr()),
        -1
    };
    data.A= &osqp_constraints_matrix;

    data.q = const_cast<double*>(objective_vector.data());
    data.l = clipped_lower_bound.data();
    data.u = clipped_upper_bound.data();
    if(workspace==0){
        osqp_setup(&workspace,&data,&settings);
        //initial_run = false;
    }
    else {
        ::c_int nnzP = objective_matrix_upper_triangle.nonZeros();
        ::c_int nnzA = constraint_matrix.nonZeros();
        ::c_int return_code = osqp_update_P_A(workspace, objective_matrix_upper_triangle.valuePtr(),
                                              OSQP_NULL, nnzP,
                                              constraint_matrix.valuePtr(),
                                              OSQP_NULL, nnzA);
        return_code = osqp_update_lin_cost(workspace, objective_vector.data());
        return_code = osqp_update_bounds(workspace, low_con->data(), high_con->data());
    }
    if (osqp_solve(workspace) != 0) {
        if (osqp_is_interrupted()) {
            tau_result.setZero();
        }
    }
    Eigen::Map<Eigen::VectorXd> solution(tau_result.data(), tau_result.size());
    if(workspace->info->status_val == OSQP_SOLVED){
        solution = Eigen::Map<const Eigen::VectorXd>(workspace->solution->x, workspace->data->n);
    }else{
        ROS_ERROR("QP does not converge");
        tau_result.setZero();
    }
}

void basePoseCallback(const free_gait_msgs::RobotStateConstPtr& robot_state){
    geometry_msgs::PoseWithCovariance base_pose_in_world_;
    base_pose_in_world_.pose = robot_state->base_pose.pose.pose;
    base_pos_rbdl[0] = robot_state->base_pose.pose.pose.position.x;
    base_pos_rbdl[1] = robot_state->base_pose.pose.pose.position.y;
    base_pos_rbdl[2] = robot_state->base_pose.pose.pose.position.z;
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

    _state._q << base_pos_rbdl[0] , base_pos_rbdl[1] , base_pos_rbdl[2],
            orient.x(), orient.y(),orient.z(),
            q(0),q(1),q(2),
            q(3),q(4),q(5),
            q(6),q(7),q(8),
            q(9),q(10),q(11),
            orient.w();
    _state._qdot << xd(0),xd(1),xd(2),omega(0),omega(1),omega(2),
            qd(0),qd(1),qd(2),
            qd(3),qd(4),qd(5),
            qd(6),qd(7),qd(8),
            qd(9),qd(10),qd(11);
    _model -> UpdateSystem(_state._q, _state._qdot);
    _model -> getCoriolis(Cori_rbdl);
    _model -> getGravity(Gravity_rbdl);
    _model->getCentroidJacobian(Jac_base);
    //Jac_rbdl.block<6,18>(0,0) = Jac_base;
    Tau_rbdl.tail(12) = tau;
    Tau_rbdl.head(6).setZero();
    contact_force = (Cori_rbdl+Gravity_rbdl-S_t*Tau_rbdl);
    C_and_G_rbdl = Cori_rbdl+Gravity_rbdl;
    //cout << "show_6_line : " << contact_force.transpose() << "\n";
    for(int i = 1; i <= 4; i++){
        _model->getFullJacobian(i,Jac_leg);
        Jac_rbdl.block<3,18>(i*3-3, 0) = Jac_leg.block<3,18>(3, 0);
        dynacore::Matrix Jac3;
        Jac3 = Jac_leg.block<3,3>(3,3+3*i);
        dynacore::Vector H_3;
        H_3 = contact_force.segment(6,3);
        //cout << "The " << i << " leg: " << Jac3.transpose().inverse()*H_3 << "\n";
    }
    //cout << "contact_force" << contact_force << "\n";
//    test_mcg.getCG(orient,q,qd,omega,xd);
//    test_mcg.getJac(q);
    dynacore::Vector force_12, qdd_rbdl;
    force_12.resize(12);
    qdd_rbdl.resize(18);
    qdd_rbdl.setZero();
    dynacore::Vect3 single_leg;
    qdd_rbdl(0) = xdd(0);
    qdd_rbdl(1) = xdd(1);
    qdd_rbdl(2) = xdd(2);
    Eigen::Matrix3d R_1 = pronto::commons::quatToRotMat(orient);
    dynacore::Vect3 gravity_world;
    gravity_world.setZero();
    gravity_world(2) = -laikago::rcg::g;
    dynacore::Vect3 gravity_base = R_1 * gravity_world;
    qdd_rbdl.segment(0,3) -= gravity_base;
    if(feet_forces.getFeetGRF(q,qd,tau,orient,feet_force_comute,qdd,xd,xdd,omega,omegad)){
//        std::cout << "RH_old : "<< feet_force_comute[laikago::FeetContactForces::LegID::RH](0)<<" "
//                  << feet_force_comute[laikago::FeetContactForces::LegID::RH](1)<<" "
//                  << feet_force_comute[laikago::FeetContactForces::LegID::RH](2)<<" "
//                  << std::endl;
        for(int i = 0; i < 4; i++){
            single_leg = feet_force_comute[i];
            force_12.segment(i*3, 3) = single_leg;
        }
        _model->getMassInertia(M_rbdl);
    }
    dynacore::Vector Jqdq_1;
    Jqdq_1.resize(3);
    Jqdq_1.setZero();
    for(int i = 1; i <= 4; i++){
        _model->getFullJDotQdot(i,Jqdq_1);
        Jqdq.segment((i-1)*3, 3) = Jqdq_1.segment(3,3);
    }

    xdd.setZero();
    if(feet_forces.getTorqueFromIDyn(q,qd,tau,orient,feet_force_comute,qdd,xd,xdd,omega,omegad)){
    }
    Calculate_Pmat_gvec(Jac_rbdl, Jqdq, contact_state, acc_cmd, force_12, L_weight, K_weight, &P_mat, &G_vec);
    double mass_laika  = 24.0;
    Calculate_constrain_mat(M_rbdl, C_and_G_rbdl, Jac_rbdl, contact_state, mass_laika,
                            0.6, &constraint_mat, &cons_lb, &cons_ub);
    //cout << "Calculate_constrains_mat has finish" << "\n";
//    P_mat's values meet my expectations, BUT not meet the requirements by osqp;
//    cout << P_mat.rows() << "--" << P_mat.cols() << "\n";
//    for(int i = 0; i < P_mat.rows(); i++){
//        cout << P_mat(i,i) << "\n";
//    }
    getTauWbc(&P_mat, &G_vec, &constraint_mat, &cons_lb, &cons_ub, qdd_f_tau_result);
    //cout << qdd_f_tau_result.size() << "\n";
    cout << "qdd int wbc :" << qdd_f_tau_result.head(18).transpose() << "\n";
    cout << "force for contact leg" << qdd_f_tau_result.segment(18,12).transpose() << "\n";
    cout << "the tau from wbc first in hitsz "<< qdd_f_tau_result.tail(12).transpose();
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg){
    xdd(0) = imu_msg->linear_acceleration.x;
    xdd(1) = imu_msg->linear_acceleration.y;
    xdd(2) = -imu_msg->linear_acceleration.z;
    //std::cout << xdd(2) << std::endl;
}
int main(int argc, char ** argv)
{
    laikago::rcg::JointState qlaika,qdlaika,qddlaika,taulaika;
    laikago::rcg::InertiaProperties inertia_prop_;
    laikago::rcg::MotionTransforms motion_transf_;
    laikago::rcg::InverseDynamics inverse_dynamics_(inertia_prop_, motion_transf_);
    laikago::rcg::ForceTransforms force_transf_;
    laikago::rcg::JSIM jsim(inertia_prop_, force_transf_);

    xdd.setZero();
    S_t.resize(18,18);
    S_t.setIdentity();
    S_t.block<6,6>(0,0).setZero();
    Tau_rbdl.resize(18);
    Jac_rbdl.resize(12,18);
    Jqdq.resize(12);
    Jqdq.setZero();
    acc_cmd.resize(18);
    acc_cmd.setZero();

    L_weight.resize(18,18);
    K_weight.resize(12,12);
    L_weight.setIdentity();
    K_weight.setIdentity();
    L_weight*=1000;

    P_mat.resize(42,42);
    P_mat.setZero();
    G_vec.resize(42);
    G_vec.setZero();

    contact_state.resize(4);
    contact_state.setOnes();

    C_and_G_rbdl.resize(18);
    C_and_G_rbdl.setZero();

    constraint_mat.resize(50,42);
    constraint_mat.setZero();
    cons_lb.resize(50);
    cons_lb.setZero();
    cons_ub.resize(50);
    cons_ub.setZero();

    qdd_f_tau_result.resize(42);
    qdd_f_tau_result.setZero();

    boost::recursive_mutex lock1;
    ros::init(argc,argv,"pronto_test_foot_force");
    ros::NodeHandle nh;
    ros::Subscriber base_sub = nh.subscribe("/gazebo/robot_states", 1 , basePoseCallback);
    ros::Subscriber imu_sub = nh.subscribe("/imu",1,imuCallback);
    ros::Publisher torqueIDyn = nh.advertise<sensor_msgs::JointState>("/joint_torque_ID",1);
    ros::Rate rate(400);
    omegad.setZero();
    xdd.setZero();
    while(ros::ok()){
        ros::spinOnce();
        sensor_msgs::JointState joint_torque;
        //std::cout << q << qd << tau << orient << qdd << xd << xdd << omega << omegad << std::endl;
        boost::recursive_mutex::scoped_lock lock(lock1);
        for(int i = 0; i < 12; i++){
            joint_torque.name.push_back("joint"+std::to_string(i));
            joint_torque.position.push_back(0);
            joint_torque.velocity.push_back(0);
            joint_torque.effort.push_back(tau(i));
        }
        torqueIDyn.publish(joint_torque);
        lock1.unlock();
        //std::cout<<q(0)<<std::endl;
        rate.sleep();
    }
    osqp_cleanup(workspace);
    return 0;
}

