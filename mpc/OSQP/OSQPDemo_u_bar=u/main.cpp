#include <OsqpEigen/OsqpEigen.h>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>

const double dt = 0.01;
const int mpc_horizon = 30;

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
};

void kinematics_update(double* pos, double vel, double angular_vel) {
    pos[0] = pos[0] + dt * vel * cos(pos[2]);
    pos[1] = pos[1] + dt * vel * sin(pos[2]);
    pos[2] = pos[2] + dt * angular_vel;
}

int controller_mpc( const double real_t, 
                    const double pos[3], 
                    double u_mpc[2]){

    OsqpEigen::Solver solver;
    int num_u = 2*mpc_horizon;
    int num_constraints = 1*mpc_horizon;

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
        matrix_constraint.insert(i, 2*i) = 1.0;
        matrix_constraint.insert(i, 2*i+1) = -1.0;
    }

    Eigen::VectorXd bound_lower(num_constraints), bound_upper(num_constraints);
    for (int i=0; i<num_constraints; i++){
        bound_lower[i] = -OsqpEigen::INFTY;
        bound_upper[i] = OsqpEigen::INFTY;
    }
    solver.data()->setLinearConstraintsMatrix(matrix_constraint);
    solver.data()->setLowerBound(bound_lower);
    solver.data()->setUpperBound(bound_upper);

    double pos_ref[5*mpc_horizon]{0};
    for (int i = 0; i < mpc_horizon; i++) {
        // circle
        pos_ref[5*i] = 3 * cos(M_PI/20 * (real_t + i * dt));
        pos_ref[5*i+1] = 3 * sin(M_PI/20 * (real_t + i * dt));
        pos_ref[5*i+2] = M_PI/2 + M_PI/20 * (real_t + i * dt);
        pos_ref[5*i+3] = 3*M_PI/20;
        pos_ref[5*i+4] = M_PI/20;

        // // line
        // pos_ref[5*i] = real_t + i * dt;
        // pos_ref[5*i+1] = real_t + i * dt;
        // pos_ref[5*i+2] = M_PI/4;
        // pos_ref[5*i+3] = sqrt(2.0);
        // pos_ref[5*i+4] = 0.0;
    }

    for (int i = 0; i < mpc_horizon; i++) {
        Eigen::Matrix3d Ai_4_A_bar = Eigen::Matrix3d::Identity();
        Eigen::Matrix3Xd Ai_4_B_bar = Eigen::Matrix3d::Identity();
        for (int j = i; j >=0; j--) {
            Eigen::Matrix3d Ai = Eigen::Matrix3d::Identity();
            Eigen::Matrix3Xd Bi = Eigen::Matrix3Xd::Zero(3, 2);
            Ai(0, 2) = -dt * pos_ref[5*j+3] * sin(pos_ref[5*j+2]);
            Ai(1, 2) = dt * pos_ref[5*j+3] * cos(pos_ref[5*j+2]);
            Ai_4_A_bar = Ai_4_A_bar * Ai;

            Bi(0, 0) = dt * cos(pos_ref[5*j+2]);
            Bi(1, 0) = dt * sin(pos_ref[5*j+2]);
            Bi(2, 1) = dt * 1.0;

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
    Eigen::VectorXd u_ref = Eigen::VectorXd::Zero(2*mpc_horizon);
    for (int i = 0; i < mpc_horizon; i++) {
        u_ref(2*i) = pos_ref[5*i+3];
        u_ref(2*i+1) = pos_ref[5*i+4];
    }
    linear_term = 2*(x_error.transpose()*A_bar.transpose()*gain_q*B_bar - u_ref.transpose()*gain_r).transpose();

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
        u_mpc[0] = solution(0);
        u_mpc[1] = solution(1);
        std::cout <<"time: " << real_t << " u_mpc: " << u_mpc[0] << ", " << u_mpc[1] << std::endl;
        return 0;
    } else {
        std::cout << "Solve failed" << std::endl;
        u_mpc[0] = 0;
        u_mpc[1] = 0;
        return 1;
    }
}

int main() {

    // // 修改增益矩阵
    // for (int index = 0; index < mpc_horizon; index++) {
    //     gain_q(3*index, 3*index) = 5.0 * 2.0; 
    // }

    std::vector<record_data> record_data_vec;
    double real_t = 0.0;
    double pos[3] = {0, -2, 42 * M_PI / 180};
    double u_mpc[2] = {0, 0};
    int time_total = 20000;

    
    for (int i=0; i<time_total; i++){
        if (controller_mpc(real_t, pos, u_mpc)) {
            std::cerr << "got error in controller_mpc func." << std::endl;
            return 1;
        }
        kinematics_update(pos, u_mpc[0], u_mpc[1]);
        record_data_vec.push_back({real_t, pos[0], pos[1], pos[2]*180/M_PI, u_mpc[0], u_mpc[1]});
        real_t += dt;
    }

    std::ofstream ofs("record_data.csv");
    ofs << "time,x,y,yaw,v,w\n"; 
    for (auto& data : record_data_vec) {
        ofs << data.time << "," 
            << data.x << "," 
            << data.y << "," 
            << data.yaw << "," 
            << data.v << "," 
            << data.w << "\n"; 
    }
    ofs.close();

    return 0;
}



