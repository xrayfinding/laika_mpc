/*
 *golaxu:
 * test osqp
 *  2021 05 28
 */

#ifndef OSQP_TEST
#define OSQP_TEST

#include "osqp.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Cholesky"
#include "eigen3/Eigen/SparseCore"
#include "eigen3/unsupported/Eigen/MatrixFunctions"
#include <vector>
#include <string>
#include <ros/ros.h>
#include "osqp/ctrlc.h"
#include "osqp/osqp.h"
#include <map>
#include "pronto_laikago_commons/feet_contact_forces.hpp"
namespace OSQPTEST {
#define OSQP_INFTY ((c_float)1e30)        // infinity

class CONTROL_MPC
{
public:
    static constexpr int K_U = 4;//quadcopter's 4 input
    static constexpr int K_X = 12;
    static constexpr int K_CONS = 5;
    CONTROL_MPC(int _N=10);//for mpc steps
    std::vector<double> compute(std::vector<double>& real_state, std::vector<double>& desire_state);
private:
    const int N_MPC;
    Eigen::VectorXd _state;
    Eigen::VectorXd _desire_state;
    Eigen::MatrixXd contact_states;
    Eigen::VectorXd umin;
    Eigen::VectorXd umax;
    Eigen::VectorXd xmin;
    Eigen::VectorXd xmax;
    Eigen::MatrixXd Ad;
    Eigen::MatrixXd Bd;
    Eigen::MatrixXd a_exp;
    Eigen::MatrixXd b_exp;
    Eigen::MatrixXd anb_exp;
    Eigen::MatrixXd a_qp;
    Eigen::MatrixXd b_qp;
    Eigen::MatrixXd anb_qp;//A^b*B (state_dim x n)* (control_dim)
    Eigen::MatrixXd p_mat;
    Eigen::MatrixXd constrains_;
    Eigen::VectorXd constrains_l;
    Eigen::VectorXd constrains_r;
    Eigen::VectorXd q_vec;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    ::OSQPWorkspace* workspace_;
    bool initial_run;
    std::vector<double> qp_solution;
};
}
#endif
