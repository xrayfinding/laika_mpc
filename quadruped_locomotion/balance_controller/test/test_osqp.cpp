#include "test/test_osqp.hpp"
namespace OSQPTEST {


// Calculates the discretized space-time dynamics. Given the dynamics equation:
//   X_dot = A X + B u
// and a timestep dt, we can estimate the snapshot of the state at t + dt by:
//   X[t + dt] = exp([A, B]dt) [X, u] = A_exp X + B_exp u
void CalculateExponentials(const Eigen::MatrixXd& a_mat,
    const Eigen::MatrixXd& b_mat, double timestep,
    Eigen::MatrixXd& ab_mat_ptr,
    Eigen::MatrixXd& a_exp_ptr,
    Eigen::MatrixXd& b_exp_ptr);

void CalculateQpMats(const Eigen::MatrixXd& a_exp, const Eigen::MatrixXd& b_exp,
    const Eigen::MatrixXd& qp_weights_single,
    const Eigen::MatrixXd& alpha_single, int horizon,
    Eigen::MatrixXd& a_qp_ptr, Eigen::MatrixXd& anb_aux_ptr,
    Eigen::MatrixXd& b_qp_ptr, Eigen::MatrixXd& p_mat_ptr);

void CalculateConstraintBounds(const Eigen::MatrixXd& contact_state, double fz_max,
    double fz_min, double friction_coeff, int horizon,
    Eigen::VectorXd& constraint_lb_ptr,
    Eigen::VectorXd& constraint_ub_ptr);

void UpdateConstraintsMatrix(
    int horizon, int num_legs,
    Eigen::MatrixXd& constraint_ptr);

CONTROL_MPC::CONTROL_MPC(int _N):
    N_MPC(_N),
    _state(K_X),
    _desire_state(K_X*_N),
    contact_states(_N, K_U),
    umin(K_U),
    umax(K_U),
    xmin(K_X),
    xmax(K_X),
    Ad(K_X, K_X),
    Bd(K_X, K_U),
    a_exp(K_X, K_X),
    b_exp(K_X,K_U),
    anb_exp(K_X+K_U,K_X+K_U),
    a_qp(K_X*_N, K_X),
    b_qp(K_X*_N, K_U*_N),
    anb_qp(K_X*_N, K_U),
    p_mat(K_U*_N, K_U*_N),
    constrains_(5*K_U*_N, K_U*_N),
    constrains_l(5*K_U*_N),
    constrains_r(5*K_U*_N),
    q_vec(K_U*_N),
    Q(K_X, K_X),
    R(K_U,K_U),
    workspace_(nullptr),
    initial_run(true),
    qp_solution(K_U*_N)
{
    _state.setZero();
    _desire_state.setZero();
    contact_states.setZero();
    umin.setZero();
    umax.setZero();
    xmin.setZero();
    xmax.setZero();
    Ad.setZero();
    Bd.setZero();
    a_exp.setZero();
    b_exp.setZero();
    anb_exp.setZero();
    a_qp.setZero();
    b_qp.setZero();
    anb_qp.setZero();
    p_mat.setZero();
    constrains_.setZero();
    constrains_l.setZero();
    constrains_r.setZero();
    Q.setZero();
    R.setZero();
    std::cout<<"init state"<<std::endl;
}
std::vector<double> CONTROL_MPC::compute(std::vector<double>& real_state, std::vector<double>& desire_state){
    Ad <<   1.,      0.,     0., 0., 0., 0., 0.1,     0.,     0.,  0.,     0.,     0.    ,
            0.,      1.,     0., 0., 0., 0., 0.,      0.1,    0.,  0.,     0.,     0.    ,
            0.,      0.,     1., 0., 0., 0., 0.,      0.,     0.1, 0.,     0.,     0.    ,
            0.0,  0.,     0., 1., 0., 0., 0.0016,  0.,     0.,  0.0992, 0.,     0.    ,
            0.,     -0.0, 0., 0., 1., 0., 0.,     -0.0016, 0.,  0.,     0.0992, 0.    ,
            0.,      0.,     0., 0., 0., 1., 0.,      0.,     0.,  0.,     0.,     0.0992,
            0.,      0.,     0., 0., 0., 0., 1.,      0.,     0.,  0.,     0.,     0.    ,
            0.,      0.,     0., 0., 0., 0., 0.,      1.,     0.,  0.,     0.,     0.    ,
            0.,      0.,     0., 0., 0., 0., 0.,      0.,     1.,  0.,     0.,     0.    ,
            0.0,  0.,     0., 0., 0., 0., 0.0,  0.,     0.,  0.0, 0.,     0.    ,
            0.,     -0.0, 0., 0., 0., 0., 0.,     -0.0, 0.,  0.,     0.0, 0.    ,
            0.,      0.,     0., 0., 0., 0., 0.,      0.,     0.,  0.,     0.,     0.0;
    Bd <<   0.,      -0.0,  0.,     0.0,
            -0.0726,  0.,      0.0726, 0.    ,
            -0.0152,  0.0152, -0.0152, 0.0152,
            -0.,     -0.0006, -0.,     0.0006,
            0.0006,   0.,     -0.0006, 0.0000,
            0.0106,   0.0106,  0.0106, 0.0106,
            0,       -1.4512,  0.,     1.4512,
            -1.4512,  0.,      1.4512, 0.    ,
            -0.3049,  0.3049, -0.3049, 0.3049,
            -0.,     -0.0236,  0.,     0.0236,
            0.0236,   0.,     -0.0236, 0.    ,
            0.2107,   0.2107,  0.2107, 0.2107;
    Eigen::VectorXd u0;
    u0.setConstant(4, 10.8);
    umin << 9.6, 9.6, 9.6, 9.6;
    umax << 13., 13., 13., 13.;
    umin = umin - u0;
    umax = umax - u0;
    xmin << -EIGEN_PI/6,-EIGEN_PI/6, -OSQP_INFTY,-OSQP_INFTY,-OSQP_INFTY,-1,
            -OSQP_INFTY,-OSQP_INFTY,-OSQP_INFTY,-OSQP_INFTY,-OSQP_INFTY,-OSQP_INFTY;
    xmax << EIGEN_PI/6,EIGEN_PI/6, OSQP_INFTY,OSQP_INFTY,OSQP_INFTY,OSQP_INFTY,
            OSQP_INFTY,OSQP_INFTY,OSQP_INFTY,OSQP_INFTY,OSQP_INFTY,OSQP_INFTY;
    Eigen::VectorXd q_diag, alpha_;
    q_diag.resize(12);
    alpha_.resize(4);
    q_diag << 0., 0., 10., 10., 10., 10., 0., 0., 0., 5., 5., 5.;
    alpha_ << .3,.3,.3,.3;
    Q = q_diag.asDiagonal();
    R.setIdentity();
    R*=0.3;
    _state << real_state[0],real_state[1],real_state[2],real_state[3],
            real_state[4],real_state[5],real_state[6],real_state[7],
            real_state[8],real_state[9],real_state[10],real_state[11];
    _desire_state << desire_state[0],desire_state[1],desire_state[2],desire_state[3],
            desire_state[4],desire_state[5],desire_state[6],desire_state[7],
            desire_state[8],desire_state[9],desire_state[10],desire_state[11];
    for (int i = 1; i < N_MPC; i++) {
        _desire_state[i*K_X + 0] = _desire_state[0];
        _desire_state[i*K_X + 1] = _desire_state[1];
        _desire_state[i*K_X + 2] = _desire_state[2];
        _desire_state[i*K_X + 3] = _desire_state[3];
        _desire_state[i*K_X + 4] = _desire_state[4];
        _desire_state[i*K_X + 5] = _desire_state[5];
        _desire_state[i*K_X + 6] = _desire_state[6];
        _desire_state[i*K_X + 7] = _desire_state[7];
        _desire_state[i*K_X + 8] = _desire_state[8];
        _desire_state[i*K_X + 9] = _desire_state[9];
        _desire_state[i*K_X + 10] = _desire_state[10];
        _desire_state[i*K_X + 11] = _desire_state[11];
    }
    ROS_INFO("pos1");
    Eigen::MatrixXd anb_mat(16,16);
    CalculateExponentials(Ad, Bd,0.25, anb_mat,a_exp,b_exp);
    const Eigen::MatrixXd qpweights = q_diag.replicate(N_MPC,1).asDiagonal();
    const Eigen::MatrixXd alpha_weights = alpha_.replicate(N_MPC,1).asDiagonal();
    ROS_INFO("pos1.1");
    CalculateQpMats(a_exp,b_exp,qpweights,alpha_weights,N_MPC,a_qp,anb_qp,b_qp,p_mat);
    ROS_INFO("pos1.1.1");
    const Eigen::MatrixXd state_diff = a_qp * _state - _desire_state;
    q_vec = 2*b_qp.transpose()*(qpweights*state_diff);
    const Eigen::VectorXd one_vec = Eigen::VectorXd::Constant(N_MPC, 1.0);
    const Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(N_MPC);
    for (int i = 0; i < K_U; i++) {
        contact_states.col(i) = zero_vec;
    }
    ROS_INFO("pos2");

    CalculateConstraintBounds(contact_states,100.0,10.0,0.6,N_MPC
                              ,constrains_l,constrains_r);
    ROS_INFO("pos2.1");
    UpdateConstraintsMatrix(N_MPC,K_U,constrains_);
    Eigen::SparseMatrix<double, Eigen::ColMajor, long long> objective_matrics = p_mat.sparseView();
    Eigen::SparseMatrix<double, Eigen::ColMajor, long long> constriant_matrics = constrains_.sparseView();
    Eigen::VectorXd objective_vector = q_vec;
    int num_variable = constrains_.cols();
    int num_constriant = constrains_.rows();
    ::OSQPSettings setting;
    osqp_set_default_settings(&setting);
    setting.verbose = false;
    setting.polish = true;
    setting.warm_start = true;
    setting.adaptive_rho_interval = 25;
    setting.eps_abs = 1e-3;
    setting.eps_rel = 1e-3;

    assert(p_mat.cols() == num_variable);
    assert(p_mat.rows() == num_variable);
    assert(q_vec.size() == num_variable);
    assert(constrains_l.size() == num_constriant);
    assert(constrains_r.size() == num_constriant);
    Eigen::VectorXd low_bound = constrains_l.cwiseMax(-OSQP_INFTY);
    Eigen::VectorXd upper_bound = constrains_r.cwiseMin(OSQP_INFTY);
    ::OSQPData data;
    data.n = num_variable;
    data.m = num_constriant;
    Eigen::SparseMatrix<double, Eigen::ColMajor, long long> obj_mat_up_triangle =
            objective_matrics.triangularView<Eigen::Upper>();
    ::csc osqp_obj_mat = {
       obj_mat_up_triangle.outerIndexPtr()[num_variable],
        num_variable,
        num_variable,
        const_cast<long long*>(obj_mat_up_triangle.outerIndexPtr()),
        const_cast<long long*>(obj_mat_up_triangle.innerIndexPtr()),
        const_cast<double*>(obj_mat_up_triangle.valuePtr()),
        -1
    };
    data.P = &osqp_obj_mat;
    ::csc osqp_cons_mat = {
        constriant_matrics.outerIndexPtr()[num_variable],
        num_constriant,
        num_variable,
        const_cast<long long*>(constriant_matrics.outerIndexPtr()),
        const_cast<long long*>(constriant_matrics.innerIndexPtr()),
        const_cast<double*>(constriant_matrics.valuePtr()),
        -1
    };
    data.A = &osqp_cons_mat;
    data.q = const_cast<double*>(objective_vector.data());
    data.l = low_bound.data();
    data.u = upper_bound.data();
    ROS_INFO("pos3");

    const int return_code = 0;
    if(workspace_ == 0){
        osqp_setup(&workspace_, &data, &setting);
        initial_run = false;
    }
    else {
        UpdateConstraintsMatrix(N_MPC, 4, constrains_);
        c_int nnzP = obj_mat_up_triangle.nonZeros();
        c_int nnzA = constriant_matrics.nonZeros();
        int return_code = osqp_update_P_A(workspace_,obj_mat_up_triangle.valuePtr(),OSQP_NULL, nnzP,
                                          constriant_matrics.valuePtr(),OSQP_NULL, nnzA);
        return_code = osqp_update_lin_cost(workspace_, objective_vector.data());
        return_code = osqp_update_bounds(workspace_, low_bound.data(), upper_bound.data());
    }
    std::vector<double> error_res;
    if(osqp_solve(workspace_)!=0){
        if(osqp_is_interrupted()){
            return error_res;
        }
    }
    Eigen::Map<Eigen::VectorXd> solution(qp_solution.data(), qp_solution.size());
    if(workspace_->info->status_val = OSQP_SOLVED){
        solution = -Eigen::Map<Eigen::VectorXd>(workspace_->solution->x, workspace_->data->n);
    }else {
        ROS_ERROR("QP_FATAL");
        return error_res;
    }
    return qp_solution;
}
void CalculateExponentials(const Eigen::MatrixXd& a_mat,
    const Eigen::MatrixXd& b_mat, double timestep,
    Eigen::MatrixXd& ab_mat_ptr,
    Eigen::MatrixXd& a_exp_ptr,
                           Eigen::MatrixXd& b_exp_ptr){
    const int state_dim = CONTROL_MPC::K_X;
    ab_mat_ptr.block<state_dim,state_dim>(0,0) = a_mat*timestep;
    const int control_dim = CONTROL_MPC::K_U;
    ab_mat_ptr.block(0,state_dim,state_dim,control_dim) = b_mat*timestep;
    Eigen::MatrixXd ab_exp = ab_mat_ptr.exp();
    a_exp_ptr = ab_exp.block<state_dim,state_dim>(0,0);
    b_exp_ptr = ab_exp.block(0,state_dim,state_dim,control_dim);
}

void CalculateQpMats(const Eigen::MatrixXd& a_exp, const Eigen::MatrixXd& b_exp,
    const Eigen::MatrixXd& qp_weights_single,
    const Eigen::MatrixXd& alpha_single, int horizon,
    Eigen::MatrixXd& a_qp_ptr, Eigen::MatrixXd& anb_aux_ptr,
    Eigen::MatrixXd& b_qp_ptr, Eigen::MatrixXd& p_mat_ptr) {
    const int state_dim = CONTROL_MPC::K_X;
    const int control_dim = CONTROL_MPC::K_U;
    a_qp_ptr.block(0,0,state_dim ,state_dim) = a_exp;
    for(int i = 1; i < horizon -1; i++){
        a_qp_ptr.block<state_dim,state_dim>(i*state_dim,0) = a_exp * a_qp_ptr.block<state_dim, state_dim>((i-1)*state_dim, 0);
    }
    anb_aux_ptr.block(0,0,state_dim,control_dim) = b_exp;
    for (int i = 1; i < horizon; i++) {
        anb_aux_ptr.block(i*state_dim, 0, state_dim, state_dim) =
                a_exp * anb_aux_ptr.block((i-1)*state_dim, 0 , state_dim, state_dim);
    }
    for (int i = 0; i < horizon; i++) {
        b_qp_ptr.block(i*state_dim, i*control_dim, state_dim, control_dim) = b_exp;
        for(int j = 0; j < i; j++){
            const int power = i - j;
            b_qp_ptr.block(i*state_dim, j * control_dim, state_dim, control_dim) =
                    anb_aux_ptr.block(power * state_dim, 0, state_dim, control_dim);
        }
    }
    ROS_INFO("func1");
    for (int i = horizon-1;i>=0;i--) {
        p_mat_ptr.block(i*control_dim, (horizon-1)*control_dim, control_dim,control_dim) = anb_aux_ptr.block((horizon-i-1)*state_dim, 0, state_dim,control_dim).transpose()
                * qp_weights_single*b_exp;
        if(i != horizon -1){
            p_mat_ptr.block((horizon-1)*control_dim, i*control_dim,control_dim,control_dim)=
                    p_mat_ptr.block(i*control_dim, (horizon-1)*control_dim, control_dim, control_dim).transpose();
        }
    }
    ROS_INFO("func2");
    for (int i = horizon - 2; i >= 0; i--) {
        p_mat_ptr.block(i * control_dim, i * control_dim, control_dim, control_dim) =
            p_mat_ptr.block((i + 1) * control_dim, (i + 1) * control_dim, control_dim,
                control_dim) +
            anb_aux_ptr.block((horizon - i - 1) * state_dim, 0, state_dim, control_dim)
            .transpose() *
            qp_weights_single *
            anb_aux_ptr.block((horizon - i - 1) * state_dim, 0, state_dim,
                control_dim);
        for (int j = i + 1; j < horizon - 1; ++j) {
            p_mat_ptr.block(i * control_dim, j * control_dim, control_dim, control_dim) =
                p_mat_ptr.block((i + 1) * control_dim, (j + 1) * control_dim, control_dim,
                    control_dim) +
                anb_aux_ptr.block((horizon - i - 1) * state_dim, 0, state_dim, control_dim)
                .transpose() *
                qp_weights_single *
                anb_aux_ptr.block((horizon - j - 1) * state_dim, 0, state_dim,
                    control_dim);
            p_mat_ptr.block(j * control_dim, i * control_dim, control_dim, control_dim) =
                p_mat_ptr.block(i * control_dim, j * control_dim, control_dim, control_dim)
                .transpose();
        }
    }
    ROS_INFO("func3");
    for(int i=0; i < horizon; i++){
        p_mat_ptr.block(i*control_dim, i*control_dim, control_dim,control_dim) += alpha_single;
    }
    p_mat_ptr *= 2.0;
}

void CalculateConstraintBounds(const Eigen::MatrixXd& contact_state, double fz_max,
    double fz_min, double friction_coeff,
    int horizon, Eigen::VectorXd& constraint_lb_ptr,
                               Eigen::VectorXd& constraint_ub_ptr){
    const int constrain_dim = CONTROL_MPC::K_CONS;
    const int control_dim = CONTROL_MPC::K_U;
    for (int i = 0; i < horizon; i++) {
        for (int j = 0; j < control_dim;j++) {
            int row = (i*control_dim + j)*constrain_dim;
            constraint_lb_ptr(row) = 0;
            constraint_lb_ptr(row+1) = 0;
            constraint_lb_ptr(row+2) = 0;
            constraint_lb_ptr(row+3) = 0;
            constraint_lb_ptr(row+4) = fz_min*contact_state(i,j);
            const double friction_ub = (friction_coeff+1)*fz_max*contact_state(i,j);
            constraint_ub_ptr(row) = friction_ub;
            constraint_ub_ptr(row+1) = friction_ub;
            constraint_ub_ptr(row+2) = friction_ub;
            constraint_ub_ptr(row+3) = friction_ub;
            constraint_ub_ptr(row+4) = fz_max*contact_state(i,j);
        }
    }
}

void UpdateConstraintsMatrix(
    int horizon, int num_legs,
                             Eigen::MatrixXd& constraint_ptr){
    for (int i = 0; i < horizon * num_legs;i++) {
        const int constrain_dim = CONTROL_MPC::K_CONS;
        constraint_ptr.block<constrain_dim, 1>(i*constrain_dim, 0)
                << -1, 1, -1, 1, 1;
    }
}


}//end namespace


int main(){
    Eigen::MatrixXd p_mat;
    p_mat.resize(2,2);
    p_mat << 4,1,1,2;
    Eigen::MatrixXd constriant;
    constriant.resize(3,2);
    constriant << 1,1,1,0,0,1;
    Eigen::SparseMatrix<double, Eigen::ColMajor, long long> obj_mat = p_mat.sparseView();
    Eigen::SparseMatrix<double, Eigen::ColMajor, long long> cons_mat= constriant.sparseView();
    Eigen::VectorXd l,u;
    l.resize(3);
    u.resize(3);
    l << 1,0,0;
    u << 1, .7,.7;
    ROS_INFO("1");

    int num_var = cons_mat.cols();
    int num_cons = cons_mat.rows();
    Eigen::SparseMatrix<double, Eigen::ColMajor, long long> obj_tri  = obj_mat.triangularView<Eigen::Upper>();
    ::csc sparse_p = {
        obj_tri.outerIndexPtr()[num_var],
        num_var,
        num_var,
        const_cast<long long*>(obj_tri.outerIndexPtr()),
        const_cast<long long*>(obj_tri.innerIndexPtr()),
        const_cast<double*>(obj_tri.valuePtr()),
        -1
    };
    ROS_INFO("1");
    ::csc sparse_A = {
        cons_mat.outerIndexPtr()[num_var],
        num_cons,
        num_var,
        const_cast<long long*>(cons_mat.outerIndexPtr()),
        const_cast<long long*>(cons_mat.innerIndexPtr()),
        const_cast<double*>(cons_mat.valuePtr()),
        -1
    };
    OSQPData data;
    data.n = num_var;
    data.m = num_cons;
    data.P = &sparse_p;
    data.A = &sparse_A;
    Eigen::VectorXd _q;
    _q.resize(2);
    _q << 1,1;
    data.q = const_cast<double*>(_q.data());
    data.l = l.data();
    data.u = u.data();
    ROS_INFO("1");

    ::OSQPSettings setting;
    osqp_set_default_settings(&setting);
    setting.verbose = false;
    setting.polish = true;
    setting.warm_start = true;
    setting.adaptive_rho_interval = 25;
    setting.eps_abs = 1e-3;
    setting.eps_rel = 1e-3;
    ROS_INFO("1");
    ::OSQPWorkspace* workspace(0);
    osqp_setup(&workspace, &data, &setting);
    osqp_solve(workspace);
    std::vector<double> qp_res(2);
    Eigen::Map<Eigen::VectorXd> solution(qp_res.data(), qp_res.size());
    solution = Eigen::Map<Eigen::VectorXd>(workspace->solution->x, workspace->data->n);
    std::cout << qp_res[0] << " - "<< qp_res[1] << std::endl;
    std::cout << "finish test" << std::endl;
    return 0;
}
