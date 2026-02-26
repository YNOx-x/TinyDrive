#include "controller.hpp"

// // 开始计时
//     std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now(); //L: debug
// // 结束计时
// std::chrono::time_point<std::chrono::system_clock> end_time = std::chrono::system_clock::now();
// std::chrono::duration<double> elapsed_seconds = end_time - start_time;
// std::cout << "L debug:  ********** time: " << elapsed_seconds.count() << "s" << std::endl; //L: debug


MpcController::MpcController(const double real_t, const double pos[])
{
    i_real_t = real_t;
    i_pos[0] = pos[0];
    i_pos[1] = pos[1];
    i_pos[2] = pos[2];

    for (int i = 0; i < mpc_horizon; i++) {
        gain_q(3*i, 3*i) = 1.0;
        gain_q(3*i+1, 3*i+1) = 1.0;
        gain_q(3*i+2, 3*i+2) = 5.0;
    }

    ref_info_vec.resize(mpc_horizon);
}

void MpcController::update_problem(const double real_t, const double pos[])
{
    i_real_t = real_t;
    i_pos[0] = pos[0];
    i_pos[1] = pos[1];
    i_pos[2] = pos[2];
    
    cal_ref_info(real_t, ref_info_vec);
    pos_error(0) = i_pos[0] - ref_info_vec[0].x;
    pos_error(1) = i_pos[1] - ref_info_vec[0].y;
    pos_error(2) = i_pos[2] - ref_info_vec[0].yaw; 

    for (int i = 0; i < mpc_horizon; i++) {
        Eigen::Matrix3d Ai_4_A_bar = Eigen::Matrix3d::Identity();
        Eigen::Matrix3Xd Ai_4_B_bar = Eigen::Matrix3d::Identity();
        for (int j = i; j >=0; j--) {
            Eigen::Matrix3d Ai = Eigen::Matrix3d::Identity();
            Eigen::Matrix3Xd Bi = Eigen::Matrix3Xd::Zero(3, 2);
            Ai(0, 2) = -1 * CTRL_FREQ * dt * ref_info_vec[j].v * sin(ref_info_vec[j].yaw);
            Ai(1, 2) = CTRL_FREQ * dt * ref_info_vec[j].v * cos(ref_info_vec[j].yaw);
            Ai_4_A_bar = Ai_4_A_bar * Ai;

            Bi(0, 0) = CTRL_FREQ * dt * cos(ref_info_vec[j].yaw);
            Bi(1, 0) = CTRL_FREQ * dt * sin(ref_info_vec[j].yaw);
            Bi(2, 1) = CTRL_FREQ * dt * 1.0;

            B_bar.block(3*i, 2*j, 3, 2) = Ai_4_B_bar * Bi;
            Ai_4_B_bar = Ai_4_B_bar * Ai;

        }
        A_bar.block(3*i, 0, 3, 3) = Ai_4_A_bar;
    }

    target_quad_iteam = B_bar.transpose() * gain_q * B_bar + gain_r;
    target_linear_iteam = pos_error.transpose()*A_bar.transpose()*gain_q*B_bar;

}

void MpcController::cal_ref_info(const double real_t, std::vector<RefInfoData>& ref_info_vec)
{
    for (int i = 0; i < mpc_horizon; i++) {
        
        // circle
        ref_info_vec[i].x = 3 * cos(M_PI/20 * (real_t + CTRL_FREQ * i * dt));
        ref_info_vec[i].y = 3 * sin(M_PI/20 * (real_t + CTRL_FREQ * i * dt));
        ref_info_vec[i].yaw = M_PI/2 + M_PI/20 * (real_t + CTRL_FREQ * i * dt);
        ref_info_vec[i].v = 3*M_PI/20;
        ref_info_vec[i].w = M_PI/20;
        kinematics_vel_2_rpm(ref_info_vec[i].v, ref_info_vec[i].w, &ref_info_vec[i].left_rpm, &ref_info_vec[i].right_rpm);
        

        // // line
        // ref_info_vec[i].x = real_t + CTRL_FREQ * i * dt;
        // ref_info_vec[i].y = real_t + CTRL_FREQ * i * dt;
        // ref_info_vec[i].yaw = M_PI/4;
        // ref_info_vec[i].v = sqrt(2.0);
        // ref_info_vec[i].w = 0.0;
        // kinematics_vel_2_rpm(ref_info_vec[i].v, ref_info_vec[i].w, &ref_info_vec[i].left_rpm, &ref_info_vec[i].right_rpm);

        // // curve
        // double dxref = 1.0;
        // double ddxref = 0;
        // double dyref = -3 * M_PI/20 * sin(M_PI/20 * (real_t + CTRL_FREQ * i * dt));
        // double ddyref = -3 * M_PI * M_PI / (20 * 20) * cos(M_PI/20 * (real_t + CTRL_FREQ * i * dt));
        // ref_info_vec[i].x = real_t + CTRL_FREQ * i * dt;
        // ref_info_vec[i].y = 3 * cos(M_PI/20 * (real_t + CTRL_FREQ * i * dt));
        // ref_info_vec[i].yaw = atan2(dyref, dxref);
        // ref_info_vec[i].v = sqrt(dxref*dxref + dyref*dyref);
        // ref_info_vec[i].w = (dxref * ddyref - dyref * ddxref) / (dxref * dxref + dyref * dyref);
        // kinematics_vel_2_rpm(ref_info_vec[i].v, ref_info_vec[i].w, &ref_info_vec[i].left_rpm, &ref_info_vec[i].right_rpm);
    }
}

bool MpcController::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                            Index& nnz_h_lag, IndexStyleEnum& index_style) 
{
    n = 2*mpc_horizon;
    m = 3*mpc_horizon;
    nnz_jac_g = 5*mpc_horizon;
    nnz_h_lag = (2*mpc_horizon + 1)*mpc_horizon;
    index_style = TNLP::C_STYLE;
    return true;
}

bool MpcController::get_bounds_info(Index n, Number* x_l, Number* x_u,
                                Index m, Number* g_l, Number* g_u) 
{
    for (Index i=0; i<n; i++) {
        x_l[i] = -2.0e19;
        x_u[i] = 2.0e19;
    }
    // 变量线性约束
    for (int i=0; i<mpc_horizon; i++) {
        g_l[3*i] = 0 - (ref_info_vec[i].v/m_radius - m_distance * ref_info_vec[i].w / (2 * m_radius));
        g_u[3*i] = m_wheels_vel_max - (ref_info_vec[i].v/m_radius - m_distance * ref_info_vec[i].w / (2 * m_radius));
        g_l[3*i+1] = 0 - (ref_info_vec[i].v/m_radius + m_distance * ref_info_vec[i].w / (2 * m_radius));
        g_u[3*i+1] = m_wheels_vel_max - (ref_info_vec[i].v/m_radius + m_distance * ref_info_vec[i].w / (2 * m_radius));
        g_l[3*i+2] = -1.0 * m_radius * m_vel_diff / m_distance - (ref_info_vec[i].w);
        g_u[3*i+2] = 1.0 * m_radius * m_vel_diff / m_distance - (ref_info_vec[i].w);
    }

    return true;
}

bool MpcController::get_starting_point(Index n, bool init_x, Number* x,
                                bool init_z, Number* z_L, Number* z_U,
                                Index m, bool init_lambda, Number* lambda) 
{
    if (init_x) {
        // std::cout << "L debug:  init x" << std::endl; //L: debug
        for (int i=0; i<mpc_horizon; i++) {
            x[2*i] = ref_info_vec[i].v;
            x[2*i+1] = ref_info_vec[i].w;
        }
    } else {
        std::cout << "L debug:  don't init x" << std::endl;//L: debug
    }
    
    return true;
}

bool MpcController::eval_f(Index n, const Number* x, bool new_x, Number& obj_value) 
{
    
    
    // Eigen::VectorXd x_vec(x);
    Eigen::Matrix<double, 60, 1> x_vec(x);

    obj_value = 0.5 * (x_vec.transpose() * target_quad_iteam * x_vec).eval().sum() + 
            (target_linear_iteam * x_vec).eval().sum();
    return true;
}

bool MpcController::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f) 
{
    // Eigen::VectorXd x_vec(x);
    Eigen::Matrix<double, 60, 1> x_vec(x);
    
    Eigen::VectorXd grad = target_quad_iteam * x_vec + target_linear_iteam.transpose();

    for (Index i=0; i<n; i++) {
        grad_f[i] = grad[i];
    }
    return true;
}

bool MpcController::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g) 
{
    for (int i = 0; i < mpc_horizon; i++) {
        g[3*i] = x[2*i]/m_radius - m_distance * x[2*i+1] / (2 * m_radius);
        g[3*i+1] = x[2*i]/m_radius + m_distance * x[2*i+1] / (2 * m_radius);
        g[3*i+2] = x[2*i+1];
    }
    return true;
}

bool MpcController::eval_jac_g(Index n, const Number* x, bool new_x,
                        Index m, Index nele_jac, Index* iRow, Index *jCol,
                        Number* values) 
{

    if (values == NULL) {
        for (int i = 0; i < mpc_horizon; i++) {
            iRow[5*i] = 3*i; jCol[5*i] = 2*i;
            iRow[5*i+1] = 3*i; jCol[5*i+1] = 2*i+1;
            iRow[5*i+2] = 3*i+1; jCol[5*i+2] = 2*i;
            iRow[5*i+3] = 3*i+1; jCol[5*i+3] = 2*i+1;
            iRow[5*i+4] = 3*i+2; jCol[5*i+4] = 2*i+1;
        }
    } else {
        for (int i = 0; i < mpc_horizon; i++) {
            values[5*i] = 1.0 / m_radius;
            values[5*i+1] = -1.0 * m_distance / (2 * m_radius);
            values[5*i+2] = 1.0 / m_radius;
            values[5*i+3] = 1.0 * m_distance / (2 * m_radius);
            values[5*i+4] = 1.0;
        }
    }
    return true;
}

// 海森矩阵结构
bool MpcController::eval_h(Index n, const Number* x, bool new_x,
                    Number obj_factor, Index m, const Number* lambda,
                    bool new_lambda, Index nele_hess, Index* iRow,
                    Index* jCol, Number* values) 
{
    if (values == NULL) {
        int num = 0;
        for (Index i = 0; i < n; i++) {
            for (Index j = 0; j <= i; j++) {
                iRow[num] = i; jCol[num] = j;
                num++;
            }
        }
    } else {
        int num = 0;
        Eigen::MatrixXd matrix_hassian = target_quad_iteam;

        for (Index i = 0; i < n; i++) {
            for (Index j = 0; j <= i; j++) {
                values[num] = matrix_hassian(i, j);
                num++;
            }
        }
    }
    return true;
}

void MpcController::finalize_solution(SolverReturn status,
                                Index n, const Number* x, const Number* z_L, const Number* z_U,
                                Index m, const Number* g, const Number* lambda,
                                Number obj_value,
                                const IpoptData* ip_data,
                                IpoptCalculatedQuantities* ip_cq) 
{
    o_mpc[0] = x[0] + ref_info_vec[0].v;
    o_mpc[1] = x[1] + ref_info_vec[0].w;

    std::cout << "Objective value = " << obj_value << std::endl;
}



