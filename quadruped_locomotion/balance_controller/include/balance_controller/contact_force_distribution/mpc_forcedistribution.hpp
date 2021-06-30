/*
*GoLaoxu:
* MPC_Class
*/

#ifndef MPC_FORCE
#define MPC_FORCE

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <assert.h>
#include <vector>
#include <assert.h>


//#ifdef _WIN32
//typedef __int64 qp_int64;
//#else
//#endif //_WIN32









#include "Eigen/Core"
#include "Eigen/SparseCore"
#include "osqp/ctrlc.h"
#include "osqp/osqp.h"
#include "unsupported/Eigen/MatrixFunctions"
#include "pronto_laikago_commons/feet_contact_forces.hpp"

namespace MPC{
using Eigen::AngleAxisd;
using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

//#define DCHECK_GT(a,b) assert((a)>(b))
//#define DCHECK_EQ(a,b) assert((a)==(b))
#define OSQP_INFTY ((c_float)1e30)        // infinity
#define OSQP_NULL 0
#define OSQP_SOLVED (1)
constexpr int k3Dim = 3;
constexpr double kGravity = 9.8;
//GoLaoXU: improve the max constrains for robot
constexpr double kMaxScale = 10;
constexpr double kMinScale = 0.1;
typedef long long qp_int64;

class ConvexMpc {
public:
    static constexpr int kStateDim =
        13;  // 6 dof pose + 6 dof velocity + 1 gravity.

    // For each foot contact force we use 4-dim cone approximation + 1 for z.
    static constexpr int kConstraintDim = 5;
//Golaoxu: alpha is the weight for force, which is 1e-6 in MIT cheetah3
    ConvexMpc(double mass, const std::vector<double>& inertia, int num_legs,
        int planning_horizon, double timestep,
        const std::vector<double>& qp_weights, double alpha = 1e-6);

    virtual ~ConvexMpc()
    {
        osqp_cleanup(workspace_);
    }
    // If not explicitly specified, we assume the quantities are measured in a
    // world frame. Usually we choose the yaw-aligned horizontal frame i.e. an
    // instanteneous world frame at the time of planning with its origin at CoM
    // and z axis aligned with gravity. The yaw-alignment means that the CoM
    // rotation measured in this frame has zero yaw component. Caveat: We expect
    // the input euler angle roll_pitch_yaw to be in ZYX format, i.e. the rotation
    // order is X -> Y -> Z, with respect to the extrinsic (fixed) coordinate
    // frame. In the intrinsic (body-attached) frame the rotation order is Z -> Y'
    // -> X".
    std::vector<double> ComputeContactForces(
        std::vector<double> com_position,
        std::vector<double> com_velocity,
        std::vector<double> com_roll_pitch_yaw,
        std::vector<double> com_angular_velocity,
        std::vector<int> foot_contact_states,
        std::vector<double> foot_positions_body_frame,
        std::vector<double> foot_friction_coeffs,
        std::vector<double> desired_com_position,
        std::vector<double> desired_com_velocity,
        std::vector<double> desired_com_roll_pitch_yaw,
        std::vector<double> desired_com_angular_velocity);

    // Reset the solver so that for the next optimization run the solver is
    // re-initialized.
    void ResetSolver();

private:
    const double mass_;
    const double inv_mass_;
    const Eigen::Matrix3d inertia_;
    const Eigen::Matrix3d inv_inertia_;
    const int num_legs_;
    const int planning_horizon_;
    const double timestep_;

    // 13 * horizon diagonal matrix.
    const Eigen::MatrixXd qp_weights_;

    // 13 x 13 diagonal matrix.
    const Eigen::MatrixXd qp_weights_single_;

    // num_legs * 3 * horizon diagonal matrix.
    const Eigen::MatrixXd alpha_;
    const Eigen::MatrixXd alpha_single_;
    const int action_dim_;

    // The following matrices will be updated for every call. However, their sizes
    // can be determined at class initialization time.
    Eigen::VectorXd state_;                 // 13
    Eigen::VectorXd desired_states_;        // 13 * horizon
    Eigen::MatrixXd contact_states_;        // horizon x num_legs
    Eigen::MatrixXd foot_positions_base_;   // num_legs x 3
    Eigen::MatrixXd foot_positions_world_;  // num_legs x 3
    Eigen::VectorXd foot_friction_coeff_;   // num_legs
    Eigen::Matrix3d rotation_;
    Eigen::Matrix3d inertia_world_;    // rotation x inertia x rotation_transpose
    Eigen::MatrixXd a_mat_;            // 13 x 13
    Eigen::MatrixXd b_mat_;            // 13 x (num_legs * 3)
    Eigen::MatrixXd ab_concatenated_;  // 13 + num_legs * 3 x 13 + num_legs * 3 :A+B
    Eigen::MatrixXd a_exp_;            // same dimension as a_mat_
    Eigen::MatrixXd b_exp_;            // same dimension as b_mat_

    // Contains all the power mats of a_exp_. Consider Eigen::SparseMatrix.
    Eigen::MatrixXd a_qp_;  // 13 * horizon x 13
    Eigen::MatrixXd b_qp_;  // 13 * horizon x num_legs * 3 * horizon sparse
    Eigen::MatrixXd b_qp_transpose_;
    Eigen::MatrixXd p_mat_;  // num_legs * 3 * horizon x num_legs * 3 * horizon
    Eigen::VectorXd q_vec_;  // num_legs * 3 * horizon vector

    // Auxiliary containing A^n*B, with n in [0, num_legs * 3)
    Eigen::MatrixXd anb_aux_;  // 13 * horizon x (num_legs * 3)

    // Contains the constraint matrix and bounds.
    Eigen::MatrixXd
        constraint_;  // 5 * num_legs * horizon x 3 * num_legs * horizon
    Eigen::VectorXd constraint_lb_;  // 5 * num_legs * horizon
    Eigen::VectorXd constraint_ub_;  // 5 * num_legs * horizon

    std::vector<double> qp_solution_;

    ::OSQPWorkspace* workspace_;
    // Whether optimizing for the first step
    bool initial_run_;
};

}

#endif
