/*
 * @file:   wbc_torque.hpp
 * @auther: Golaoxu
 * @data:   September 15, 2021
 * @brief   This file translate multiple tasks control problem into a QP problem
 *           and solve it use OSQP.(WBC)
 * @attention: All the kinematic & dynamics states are described in world frame !!!!!!!!!!
*/
#pragma once
#include "ros/ros.h"
#include <vector>
#include <string>
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/LU>
#include "osqp/ctrlc.h"
#include "osqp/osqp.h"
#include "unsupported/Eigen/MatrixFunctions"
#include "model/DynModel.hpp"
#include "model/KinModel.hpp"
#include "model/RobotDefinition.hpp"
#include "model/RobotModel.hpp"
#include "utils/pseudo_inverse.hpp"
#include "utils/common_convert_calcu.hpp"
#include "pronto_quadruped_commons/transforms.h"
#include "pronto_quadruped_commons/geometry/rotations.h"
#include "free_gait_core/TypeDefs.hpp"
using namespace dynacore;
using namespace std;
using Eigen::VectorXd;
#define OSQP_INFTY ((c_float)1e30)        // infinity
#define OSQP_NULL 0
#define OSQP_SOLVED (1)
class WholeBodyControlOsqp{
public:
    /*
     * Golaoxu :
     *  In RBDL, all the states are discribed in world frame !!!!!!
     */
    WholeBodyControlOsqp(const VectorXd& q_robot,const VectorXd& qdot_robot,const VectorXd& contacts_robot,const VectorXd& force_robot,
                         double mass,int state_dim,double hessian_weight, double gradient_weight);
    virtual ~WholeBodyControlOsqp(){
        osqp_cleanup(workspace_);
    }
    void calculate_acc_cmd(const Eigen::VectorXd& p, const Eigen::VectorXd& pdot, const Eigen::VectorXd& p_target, const Eigen::VectorXd& pdot_target,
                           const double delta_t_wbc ,const double kp, const double kd);

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
                             const Eigen::MatrixXd& k_weight, Eigen::MatrixXd* P_mat_ptr, Eigen::VectorXd* G_vec_ptr);

    /*Golaoxu:
     * Jsim: joint state inertial matrix;
     * C_G:  C(coriolis and centrifugal force) and Gravity;
    */
    void Calculate_constrain_mat(const Eigen::MatrixXd& Jsim,const Eigen::VectorXd C_G ,const Eigen::MatrixXd& Jac_1218,
                                 const Eigen::VectorXd& contact_state, const double mass,double friction_coeff,
                                 Eigen::MatrixXd* C_mat, Eigen::VectorXd* low_con, Eigen::VectorXd* high_con);

    //Update the dyanmic model before compute the tau;
    void update_robot_system(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, const Eigen::VectorXd& f_mpc,
                             const Eigen::VectorXd& contact_state);

    //Compute final result:
    void computeTauWbc(const Eigen::MatrixXd* Pmat_ptr, const Eigen::VectorXd* Gvec_ptr,
                   const Eigen::MatrixXd* Cmat_ptr, const Eigen::VectorXd* low_con,
                   const Eigen::VectorXd* high_con, Eigen::VectorXd& tau_result);

    /*  Golaoxu :
     *  Interface func
     *  this functions should be called after the update_robot_system
     *
     */
    void computeTau(Eigen::VectorXd& tau);

    /*
     * Golaoxu :
     * Only use this function can finish all the job for compute the tau.
    */
    void update_Sys_and_getTau(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, const Eigen::VectorXd& f_mpc,
                               const Eigen::VectorXd& contact_state,Eigen::VectorXd& tau,
                               const Eigen::VectorXd& q_tar, const Eigen::VectorXd &qd_tar);

private:
    //Matrix for rigid body dynamic:
    std::shared_ptr<RobotModel> _model;
    Matrix M_rbdl;//18 x 18: Joint Space Inertial Matrix
    dynacore::Vector Cori_rbdl, Gravity_rbdl, Tau_rbdl, C_and_G;//18x1
    Matrix Jac_base;//not use here;
    Matrix Jac_leg;//6x18 : upper 3 line is rotation; low 3 line is translation;
    Matrix Jac_rbdl;//12*18 : collection the translation of 4 legs;
    dynacore::Vector Jdqd;

    //Matrix for solving the quadratic programming:
    ::OSQPWorkspace* workspace_;
    Eigen::MatrixXd K_weight;//weight for MPC_force
    Eigen::MatrixXd L_weight;//weight for a_cmd
    //costfunc : 0.5*x'Px+g'x
    Eigen::MatrixXd P_mat;
    Eigen::VectorXd G_vec;
    //constrains: lb <= Ax <= ub;
    Eigen::MatrixXd Constraint_mat;
    Eigen::VectorXd cons_lb;
    Eigen::VectorXd cons_ub;

    //osqp results
    Eigen::VectorXd qdd_f_tau_result;

    //others
    Eigen::VectorXd _q;
    Eigen::VectorXd _qdot;
    Eigen::VectorXd _contact_state;
    Eigen::VectorXd acc_cmd;
    Eigen::VectorXd force_mpc;
    double friction_coeff;
    double mass_robot;
};
