/*
 * @file:   wbc_torque.cpp
 * @auther: Golaoxu
 * @data:   September 15, 2021
 * @brief   This file translate multiple tasks control problem into a QP problem
 *           and solve it use OSQP.
*/
#include "balance_controller/contact_force_distribution/wbc_torque.hpp"
#include "pronto_quadruped_commons/geometry/rotations.h"

WholeBodyControlOsqp::WholeBodyControlOsqp(const VectorXd& q_robot,
    const VectorXd& qdot_robot,const VectorXd& contacts_robot,const VectorXd& force_robot,double mass, int state_dim,
    double hessian_weight, double gradient_weight)
    :_q(q_robot),
     _qdot(qdot_robot),
     _contact_state(contacts_robot),
     force_mpc(force_robot),
     mass_robot(mass),
     M_rbdl(state_dim,state_dim),
     Cori_rbdl(state_dim),
     Gravity_rbdl(state_dim),
     Tau_rbdl(state_dim),
     C_and_G(state_dim),
     Jac_base(6,state_dim),
     Jac_leg(6,state_dim),
     Jac_rbdl(12,state_dim),
     Jdqd(12),
     workspace_(0),
     K_weight(12,12),
     L_weight(state_dim,state_dim),
     P_mat(42,42),
     G_vec(42),
     Constraint_mat(50,42),
     cons_lb(50),
     cons_ub(50),
     qdd_f_tau_result(42),
     acc_cmd(state_dim),
     friction_coeff(0.6)
{
    _model.reset(new RobotModel());
    M_rbdl.setZero();
    Cori_rbdl.setZero();
    Gravity_rbdl.setZero();
    Tau_rbdl.setZero();
    C_and_G.setZero();
    Jac_base.setZero();
    Jac_leg.setZero();
    Jac_rbdl.setZero();
    Jdqd.setZero();

    K_weight.setIdentity();
    K_weight*=gradient_weight;
    L_weight.setIdentity();
    L_weight*=hessian_weight;
    P_mat.setZero();
    G_vec.setZero();
    Constraint_mat.setZero();
    cons_lb.setZero();
    cons_ub.setZero();
    qdd_f_tau_result.setZero();
    acc_cmd.setZero();
}

/*
 * Golaoxu :
 *      calculate the desired acc for the task;
 *      Needed count before solve the optimization problem;
*/
void WholeBodyControlOsqp::calculate_acc_cmd(const Eigen::VectorXd &p, const Eigen::VectorXd &pdot, const Eigen::VectorXd &p_target, const Eigen::VectorXd &pdot_target,
                                             const double delta_t_wbc, const double kp, const double kd){
    //TODO : finish this function;
    /*Attention:
     *  In p_des , the foot_pos_in_world dont count the base_CoM_in_world !!!( -> real_foot_pos-base_in_world )
     */
    //1: calculate the pos error in the world frame ---> p_error;
    RotationQuaternion quat_des, quat_real;
    quat_des.x() = p_target(3);
    quat_des.y() = p_target(4);
    quat_des.z() = p_target(5);
    quat_des.w() = p_target(18);
    quat_real.x() = p(3);
    quat_real.y() = p(4);
    quat_real.z() = p(5);
    quat_real.w() = p(18);
    Eigen::VectorXd orientationError;
    RotationQuaternion quat_des1 = quat_des.inverted();
    orientationError = -quat_des1.boxMinus(quat_real.inverted());
    //TODO: golaoxu
    //test the orientation error in the frame world.
    Eigen::VectorXd p_error,pdot_error;
    p_error.resize(18);
    pdot_error.resize(18);
    p_error.segment(0,3) = p_target.segment(0,3) - p.segment(0,3);
    p_error.segment(3,3) = orientationError;
    dynacore::Vect3 foot_pos_world, base_CoM_world;
    base_CoM_world = p.segment(0,3);
    Eigen::VectorXd foot_pos_all;
    foot_pos_all.resize(12);
    for(int i = 1; i <= 4; i++){
        _model->getPos(i, foot_pos_world);
        foot_pos_world-=base_CoM_world;
        foot_pos_all.segment(i*3-3, 3) = foot_pos_world;
    }
    p_error.segment(6,12) = p_target.segment(6,12) - foot_pos_all;
    //2: calculate the vel error in the world frame ---> pdot_error;
    pdot_error.segment(0,6) = pdot_target.segment(0,6) - pdot.segment(0,6);
    /*
     * Golaoxu : this has a problems that the frame are not in the world
     *   Translate the velocity from the floating base frame into the fixed world frame;
     *   Equation : v_world = v_base_in_world + (v_foot_in_base)_in_world
     *              v_base_in_world = tans_in_world + (ang_in_base x p_foot_base)_in_world;
     *   Attention: the vel perhaps not ok!!! ->(if not ok) change the PD controller, remove the D term(for the foot vel);
     */
    Eigen::VectorXd ang_vel;
    Eigen::Vector3d ang_vel_base;
    ang_vel = pdot.segment(3,3);
    Eigen::Quaterniond world2base = quat_real.inverted().toImplementation();
    Eigen::Quaterniond base2world = quat_real.toImplementation();
    ang_vel_base = world2base * ang_vel;
    Eigen::VectorXd vel_foot_base, foot_pos_base, vel_tar_world, vel_real_world;
    vel_foot_base.resize(12);
    foot_pos_base.resize(12);
    vel_tar_world.resize(12);
    Eigen::Vector3d singleFootposBasetmp;
    for(int i = 0; i < 4; i++){
        foot_pos_base.segment(i*3, 3) = world2base * foot_pos_all.segment(i*3,3);
        singleFootposBasetmp = foot_pos_base.segment(i*3, 3);
        vel_foot_base.segment(i*3,3) = ang_vel_base.cross(singleFootposBasetmp);
        vel_tar_world.segment(i*3,3) = base2world*vel_foot_base + pdot.segment(0,3) + pdot_target.segment(i*3+6,3);
    }
    vel_real_world.resize(12);
    dynacore::Matrix Jac6_18;
    for(int i = 0; i < 4; i++){
        _model->getFullJacobian(i+1, Jac6_18);
        vel_real_world.segment(i*3, 3) = Jac6_18.block<3,18>(3,0) * pdot;
    }
    pdot_error.segment(6,12) = vel_tar_world - vel_real_world;
    Eigen::VectorXd pdot_task;
    pdot_task.resize(18);
    pdot_task.segment(0,6) = pdot.segment(0,6);
    pdot_task.segment(6,12) = vel_real_world;
    //Golaoxu : set the foot vel error to 0 for the time being.
    acc_cmd = (kp*(p_error) + delta_t_wbc*kd*pdot_error + delta_t_wbc*kp*pdot_task)/(1+kp*pow(delta_t_wbc, 2)+kd*delta_t_wbc);
    //Golaoxu : set the feedforward to 0;
    acc_cmd = (kp*(p_error) + delta_t_wbc*kd*pdot_error)/(1+kp*pow(delta_t_wbc, 2)+kd*delta_t_wbc);
    cout << acc_cmd.transpose() << "\n";
    //acc_cmd = -(kp*(p-p_target) + delta_t_wbc*kd*(pdot-pdot_target) + delta_t_wbc*kp*pdot)/(1+kp*pow(delta_t_wbc, 2)+kd*delta_t_wbc);
}
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

void WholeBodyControlOsqp::Calculate_Pmat_gvec(const dynacore::Matrix &Jac_1218, const dynacore::Vector jdqd, const Eigen::VectorXd &contact_state, const Eigen::VectorXd &acc_cmd, const Eigen::VectorXd &f_mpc,
                                               const Eigen::MatrixXd &l_weight, const Eigen::MatrixXd &k_weight, Eigen::MatrixXd *P_mat_ptr, Eigen::VectorXd *G_vec_ptr){
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
 *
 * Jsim: joint state inertial matrix;
 * C_G: C(coriolis and centrifugal force) and Gravity;
*/
void WholeBodyControlOsqp::Calculate_constrain_mat(const Eigen::MatrixXd &Jsim, const Eigen::VectorXd C_G, const Eigen::MatrixXd &Jac_1218, const Eigen::VectorXd &contact_state,
                                                   const double mass, double friction_coeff, Eigen::MatrixXd *C_mat, Eigen::VectorXd *low_con, Eigen::VectorXd *high_con){
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
    return;
}

void WholeBodyControlOsqp::update_robot_system(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot, const Eigen::VectorXd& f_mpc,
                                               const Eigen::VectorXd &contact_state)
{
    /*Update the system finished in computerTauWbc
     * and this func should be set as privated
     */
//    _q = q;
//    _qdot = qdot;
//    _model->UpdateSystem(_q, _qdot);
    _model->getCoriolis(Cori_rbdl);
    _model->getGravity(Gravity_rbdl);
    _model->getCentroidJacobian(Jac_base);
    C_and_G = Cori_rbdl+Gravity_rbdl;
    dynacore::Vector Jdqd_leg;
    for (int i = 1; i <= 4; i++) {
        _model->getFullJacobian(i,Jac_leg);
        Jac_rbdl.block<3,18>(i*3-3,0) = Jac_leg.block<3,18>(3,0);
        _model->getFullJDotQdot(i, Jdqd_leg);
        Jdqd.segment((i-1)*3, 3) = Jdqd_leg.segment(3,3);
    }

    Eigen::Quaterniond orient1;
    orient1.w() = _q(18);
    orient1.x() = _q(3);
    orient1.y() = _q(4);
    orient1.z() = _q(5);
    Eigen::Matrix3d R_1 = pronto::commons::quatToRotMat(orient1);
    _model->getMassInertia(M_rbdl);
    force_mpc = f_mpc;
    _contact_state = contact_state;
    Calculate_Pmat_gvec(Jac_rbdl, Jdqd, _contact_state, acc_cmd, force_mpc,L_weight,K_weight, &P_mat,&G_vec);
    Calculate_constrain_mat(M_rbdl,C_and_G,Jac_rbdl,_contact_state,mass_robot,friction_coeff,&Constraint_mat, &cons_lb,&cons_ub);
    return;
}

void WholeBodyControlOsqp::computeTauWbc(const Eigen::MatrixXd *Pmat_ptr, const Eigen::VectorXd *Gvec_ptr, const Eigen::MatrixXd *Cmat_ptr, const Eigen::VectorXd *low_con, const Eigen::VectorXd *high_con, Eigen::VectorXd &tau_result){
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
    if(workspace_==0){
        osqp_setup(&workspace_,&data,&settings);
        //initial_run = false;
    }
    else {
        ::c_int nnzP = objective_matrix_upper_triangle.nonZeros();
        ::c_int nnzA = constraint_matrix.nonZeros();
        ::c_int return_code = osqp_update_P_A(workspace_, objective_matrix_upper_triangle.valuePtr(),
                                              OSQP_NULL, nnzP,
                                              constraint_matrix.valuePtr(),
                                              OSQP_NULL, nnzA);
        return_code = osqp_update_lin_cost(workspace_, objective_vector.data());
        return_code = osqp_update_bounds(workspace_, low_con->data(), high_con->data());
    }
    if (osqp_solve(workspace_) != 0) {
        if (osqp_is_interrupted()) {
            tau_result.setZero();
        }
    }
    Eigen::Map<Eigen::VectorXd> solution(tau_result.data(), tau_result.size());
    if(workspace_->info->status_val == OSQP_SOLVED){
        solution = Eigen::Map<const Eigen::VectorXd>(workspace_->solution->x, workspace_->data->n);
    }else{
        ROS_ERROR("QP does not converge");
        tau_result.setZero();
    }
}


void WholeBodyControlOsqp::computeTau(Eigen::VectorXd& tau){
    computeTauWbc(&P_mat, &G_vec, &Constraint_mat, &cons_lb, &cons_ub, tau);
    return;
}


void WholeBodyControlOsqp::update_Sys_and_getTau(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot,
                                                 const Eigen::VectorXd &f_mpc, const Eigen::VectorXd &contact_state, Eigen::VectorXd &tau,
                                                 const Eigen::VectorXd &q_tar, const Eigen::VectorXd &qd_tar){
    /*
     * Golaoxu:
     *  UpdateSys should be finished before calculate the acc_cmd;
     *  calculate the acc_cmd should be finished before update_robot_system;
     *  Update_robot_system shouldn't used alone;
     */
    _q = q;
    _qdot = qdot;
    _model->UpdateSystem(_q,_qdot);
    calculate_acc_cmd(q,qdot,q_tar,qd_tar,0.0025,100.0,10.0);
    //update_robot_system(q, qdot, f_mpc, contact_state);
    //computeTau(tau);
    return;
}
