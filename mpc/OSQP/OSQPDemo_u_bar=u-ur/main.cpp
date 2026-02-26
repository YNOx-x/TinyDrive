#include <OsqpEigen/OsqpEigen.h>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>

#define CTRL_FREQ 5 // Hz= 100 / CTRL_FREQ
int ctrl_index = 0;

const double dt = 0.01;
const int mpc_horizon = 30;

const double m_distance = 0.35;
const double m_radius = 0.05;
const double m_vel_diff = 2.0;
const double m_wheels_vel_max = 25; // OsqpEigen::INFTY; // 

Eigen::MatrixXd gain_q = 1.0 * 1.0 * Eigen::MatrixXd::Identity(3*mpc_horizon, 3*mpc_horizon);
Eigen::MatrixXd gain_r = 1.0 * 1.0 * Eigen::MatrixXd::Identity(2*mpc_horizon, 2*mpc_horizon);

Eigen::MatrixXd A_bar = Eigen::MatrixXd::Zero(3*mpc_horizon, 3);
Eigen::MatrixXd B_bar = Eigen::MatrixXd::Zero(3*mpc_horizon, 2*mpc_horizon);

struct record_data {
    double time;

    double x;
    double y;
    double yaw;

    double v;
    double w;

    double left_rpm;
    double right_rpm;

    double e_x;
    double e_y;
};

void kinematics_update(double* pos, double vel, double angular_vel) {
    pos[0] = pos[0] + dt * vel * cos(pos[2]);
    pos[1] = pos[1] + dt * vel * sin(pos[2]);
    pos[2] = pos[2] + dt * angular_vel;
}

void kinematics_vel_2_rpm(double vel, double angular_vel, double* left, double* right) {
    *left = vel / m_radius - angular_vel * m_distance / (2 * m_radius);
    *right = vel / m_radius + angular_vel * m_distance / (2 * m_radius);
}

void kinematics_rpm_2_vel(double left, double right, double* vel, double* angular_vel) {
    *vel = (left + right) * m_radius / 2.0 ;
    *angular_vel = (right - left) * m_radius / m_distance ;
}

void get_ref_info(double real_t, double ref_info[]){
    double temp_pos_ref[2*2]{0};
    for (int i = 0; i < 2; i++) {
        // // circle
        // temp_pos_ref[2*i] = 3 * cos(M_PI/20 * (real_t + i * dt));
        // temp_pos_ref[2*i+1] = 3 * sin(M_PI/20 * (real_t + i * dt));

        // // line
        // temp_pos_ref[2*i] = real_t + i * dt;
        // temp_pos_ref[2*i+1] = real_t + i * dt;

        // curve
        temp_pos_ref[2*i] = real_t + i * dt;
        temp_pos_ref[2*i+1] = 3 * cos(M_PI/20 * (real_t + i * dt));

    }
    double temp_vel_x = (temp_pos_ref[2] - temp_pos_ref[0])/dt;
    double temp_vel_y = (temp_pos_ref[3] - temp_pos_ref[1])/dt;

    // // liujw
    // ref_info[0] = temp_pos_ref[0];
    // ref_info[1] = temp_pos_ref[1];
    // daiy
    ref_info[0] = temp_pos_ref[2];
    ref_info[1] = temp_pos_ref[3];

    ref_info[2] = atan2(temp_vel_y, temp_vel_x);

}

int controller_mpc( const double real_t, 
                    const double pos[3], 
                    double u_mpc[2]){

    OsqpEigen::Solver solver;
    int num_u = 2*mpc_horizon;
    int num_constraints = 3*mpc_horizon;

    // reference info
    double pos_ref[5*mpc_horizon]{0};
    for (int i = 0; i < mpc_horizon; i++) {
        // // circle
        // pos_ref[5*i] = 3 * cos(M_PI/20 * (real_t + CTRL_FREQ * i * dt));
        // pos_ref[5*i+1] = 3 * sin(M_PI/20 * (real_t + CTRL_FREQ * i * dt));
        // pos_ref[5*i+2] = M_PI/2 + M_PI/20 * (real_t + CTRL_FREQ * i * dt);
        // pos_ref[5*i+3] = 3*M_PI/20;
        // pos_ref[5*i+4] = M_PI/20;

        // // line
        // pos_ref[5*i] = real_t + CTRL_FREQ * i * dt;
        // pos_ref[5*i+1] = real_t + CTRL_FREQ * i * dt;
        // pos_ref[5*i+2] = M_PI/4;
        // pos_ref[5*i+3] = sqrt(2.0);
        // pos_ref[5*i+4] = 0.0;

        // curve
        double dxref = 1.0;
        double ddxref = 0;
        double dyref = -3 * M_PI/20 * sin(M_PI/20 * (real_t + CTRL_FREQ * i * dt));
        double ddyref = -3 * M_PI * M_PI / (20 * 20) * cos(M_PI/20 * (real_t + CTRL_FREQ * i * dt));
        pos_ref[5*i] = real_t + CTRL_FREQ * i * dt;
        pos_ref[5*i+1] = 3 * cos(M_PI/20 * (real_t + CTRL_FREQ * i * dt));
        pos_ref[5*i+2] = atan2(dyref, dxref);
        pos_ref[5*i+3] = sqrt(dxref*dxref + dyref*dyref);
        pos_ref[5*i+4] = (dxref * ddyref - dyref * ddxref) / (dxref * dxref + dyref * dyref);


    }

    // 初始化求解器
    solver.settings()->setVerbosity(true);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(num_u);
    solver.data()->setNumberOfConstraints(num_constraints);

    // 初始化目标函数矩阵
    Eigen::SparseMatrix<double> matrix_hessian(num_u, num_u);

    // 初始化线性项
    Eigen::VectorXd linear_term(num_u);

    // 初始化并设置约束矩阵及其边界
    Eigen::SparseMatrix<double> matrix_constraint(num_constraints, num_u);
    for (int i=0; i<mpc_horizon; i++){
        matrix_constraint.insert(3*i, 2*i) = 1.0 / m_radius;
        matrix_constraint.insert(3*i, 2*i + 1) = -1.0 * m_distance / (2 *m_radius);
        matrix_constraint.insert(3*i+1, 2*i) = 1.0 / m_radius;
        matrix_constraint.insert(3*i+1, 2*i + 1) = 1.0 * m_distance / (2 *m_radius);
        matrix_constraint.insert(3*i+2, 2*i + 1) = 1.0;
    }

    Eigen::VectorXd bound_lower(num_constraints), bound_upper(num_constraints);
    for (int i=0; i<mpc_horizon; i++){
        // 施加约束
        bound_lower[3*i] = 0 - (pos_ref[5*i+3]/m_radius - m_distance * pos_ref[5*i+4]/ (2 * m_radius));
        bound_upper[3*i] = m_wheels_vel_max - (pos_ref[5*i+3]/m_radius - m_distance * pos_ref[5*i+4]/ (2 * m_radius)); 
        bound_lower[3*i+1] = 0- (pos_ref[5*i+3]/m_radius + m_distance * pos_ref[5*i+4]/ (2 * m_radius));
        bound_upper[3*i+1] = m_wheels_vel_max- (pos_ref[5*i+3]/m_radius + m_distance * pos_ref[5*i+4]/ (2 * m_radius));
        bound_lower[3*i+2] = -m_radius * m_vel_diff / m_distance - pos_ref[5*i+4];
        bound_upper[3*i+2] = m_radius * m_vel_diff / m_distance - pos_ref[5*i+4];
        // bound_lower[3*i+2] = -OsqpEigen::INFTY;
        // bound_upper[3*i+2] = OsqpEigen::INFTY;

        // // 无约束
        // bound_lower[3*i] = -OsqpEigen::INFTY;
        // bound_upper[3*i] = OsqpEigen::INFTY;
        // bound_lower[3*i+1] = -OsqpEigen::INFTY;
        // bound_upper[3*i+1] = OsqpEigen::INFTY;
        // bound_lower[3*i+2] = -OsqpEigen::INFTY;
        // bound_upper[3*i+2] = OsqpEigen::INFTY;
    }
    solver.data()->setLinearConstraintsMatrix(matrix_constraint);
    solver.data()->setLowerBound(bound_lower);
    solver.data()->setUpperBound(bound_upper);


    for (int i = 0; i < mpc_horizon; i++) {
        Eigen::Matrix3d Ai_4_A_bar = Eigen::Matrix3d::Identity();
        Eigen::Matrix3Xd Ai_4_B_bar = Eigen::Matrix3d::Identity();
        for (int j = i; j >=0; j--) {
            Eigen::Matrix3d Ai = Eigen::Matrix3d::Identity();
            Eigen::Matrix3Xd Bi = Eigen::Matrix3Xd::Zero(3, 2);
            Ai(0, 2) = -1 * CTRL_FREQ * dt * pos_ref[5*j+3] * sin(pos_ref[5*j+2]);
            Ai(1, 2) = CTRL_FREQ * dt * pos_ref[5*j+3] * cos(pos_ref[5*j+2]);
            Ai_4_A_bar = Ai_4_A_bar * Ai;

            Bi(0, 0) = CTRL_FREQ * dt * cos(pos_ref[5*j+2]);
            Bi(1, 0) = CTRL_FREQ * dt * sin(pos_ref[5*j+2]);
            Bi(2, 1) = CTRL_FREQ * dt * 1.0;

            B_bar.block(3*i, 2*j, 3, 2) = Ai_4_B_bar * Bi;
            Ai_4_B_bar = Ai_4_B_bar * Ai;

        }
        A_bar.block(3*i, 0, 3, 3) = Ai_4_A_bar;
    }

    matrix_hessian = (B_bar.transpose() * gain_q * B_bar + gain_r).sparseView();

    Eigen::VectorXd x_error = Eigen::VectorXd::Zero(3);
    x_error(0) = pos[0] - pos_ref[0];
    x_error(1) = pos[1] - pos_ref[1];
    x_error(2) = pos[2] - pos_ref[2];
    linear_term = 2*(x_error.transpose()*A_bar.transpose()*gain_q*B_bar).transpose();

    if (!solver.data()->setHessianMatrix(matrix_hessian)) return 1;
    if (!solver.data()->setGradient(linear_term)) return 1;    

    if (!solver.initSolver()) {
        std::cerr << "Failed to initialize solver!" << std::endl;
        return 1;
    }

    solver.solveProblem();

    if (solver.getStatus() == OsqpEigen::Status::Solved) {
        Eigen::VectorXd solution = solver.getSolution();
        // std::cout << "Optimal solution:\n" << solution << std::endl;
        // std::cout << "Objective value: " << 0.5*solution.dot(matrix_hessian*solution) + linear_term.dot(solution) << std::endl;
        u_mpc[0] = solution(0) + pos_ref[3];
        u_mpc[1] = solution(1) + pos_ref[4];

        // // trick
        // if (x_error(0)*x_error(0) + x_error(1)*x_error(1) > 0.5) {
        //     u_mpc[0] = u_mpc[0] + 0.001 * (2*rand() - 1);
        //     u_mpc[1] = u_mpc[1] + 0.001 * (2*rand() - 1);
        // }
        std::cout <<"Optimal value: "<<solver.getObjValue()<< std::endl;
        std::cout <<"time: " << real_t << " u_mpc: " << u_mpc[0] << ", " << u_mpc[1] << std::endl;
        std::cout <<"X: " << pos[0] << ", Y " << pos[1] <<  std::endl;
        return 0;
    } else {
        std::cout << "Solve failed" << std::endl;
        u_mpc[0] = 0;
        u_mpc[1] = 0;
        return 1;
    }
}

int main() {

    // 修改增益矩阵
    for (int index = 0; index < mpc_horizon; index++) {
        gain_q(3*index, 3*index) = 5.0 * 1.0; 
    }

    std::vector<record_data> record_data_vec;
    double real_t = 0.0;
    double pos[3] = {-0.5, 3.5, -10 * M_PI / 180};
    double u_mpc[2] = {0, 0};
    int time_total = 20000;
    double ref_info[3];

    
    for (int i=0; i<time_total; i++){
        if (ctrl_index == 0) {
            if (controller_mpc(real_t, pos, u_mpc)) {
                std::cerr << "got error in controller_mpc func." << std::endl;
                return 1;
            }
        }
        kinematics_update(pos, u_mpc[0], u_mpc[1]);

        get_ref_info(real_t, ref_info);
        double left_rpm, right_rpm;
        kinematics_vel_2_rpm(u_mpc[0], u_mpc[1], &left_rpm, &right_rpm);
        record_data_vec.push_back({real_t, pos[0], pos[1], pos[2]*180/M_PI, u_mpc[0], u_mpc[1], left_rpm, right_rpm, 
                                   pos[0]-ref_info[0], pos[1]-ref_info[1]});

        real_t += dt;
        ctrl_index = (ctrl_index + 1) % CTRL_FREQ;
    }

    std::ofstream ofs("record_data.csv");
    ofs << "time,x,y,yaw,v,w,left_rpm,right_rpm,e_x,e_y\n"; 
    for (auto& data : record_data_vec) {
        ofs << data.time << "," 
            << data.x << "," 
            << data.y << "," 
            << data.yaw << "," 
            << data.v << "," 
            << data.w << ","
            << data.left_rpm << ","
            << data.right_rpm << ","
            << data.e_x << ","
            << data.e_y << "\n"; 
    }
    ofs.close();

    return 0;
}



