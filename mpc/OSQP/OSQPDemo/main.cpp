#include <OsqpEigen/OsqpEigen.h>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>

using namespace Eigen; 


// Kronecker积实现
Eigen::MatrixXd kroneckerProduct(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B) {
    Eigen::MatrixXd C(A.rows()*B.rows(), A.cols()*B.cols());
    for(int i=0; i<A.rows(); ++i)
        for(int j=0; j<A.cols(); ++j)
            C.block(i*B.rows(), j*B.cols(), B.rows(), B.cols()) = A(i,j)*B;
    return C;
}



//matlab 转 cpp
using namespace Eigen;  

// 角度限制 [-π, π]  
double angle_bound(double theta) {  
    return theta - 2 * M_PI * std::floor((theta + M_PI) / (2 * M_PI));  
}  

int main() {  
    // 参数定义  
    const int steps = 20000;  
    const double dt = 0.01;  
    const int N = 30; // 预测步长  
    const int n = 3;  // 状态变量维度 (x, y, theta)  
    const int p = 2;  // 控制变量维度 (v, w)  

    // 定义参考轨迹  
    VectorXd x_ref(steps), y_ref(steps), theta_ref(steps), v_ref(steps), w_ref(steps);  
    for (int i = 0; i < steps; ++i) {  
        // x_ref(i) = 2 * cos(0.5 * (i + 1) * dt);  
        // y_ref(i) = sin(0.5 * (i + 1) * dt);  
        x_ref(i) = 3 * cos(M_PI/20 * (i) * dt);
        y_ref(i) = 3 * sin(M_PI/20 * (i) * dt);
    }  

    // 计算速度 v 和角速度 w  
    for (int i = 0; i < steps - 1; ++i) {  
        double vx = (x_ref(i + 1) - x_ref(i)) / dt;  
        double vy = (y_ref(i + 1) - y_ref(i)) / dt;  
        v_ref(i) = sqrt(vx * vx + vy * vy);  
        theta_ref(i) = atan2(vy, vx);  
    }  
    v_ref(steps - 1) = v_ref(steps - 2);  
    theta_ref(steps - 1) = theta_ref(steps - 2);  
    for (int i = 0; i < steps - 1; ++i) {  
        w_ref(i) = angle_bound(theta_ref(i + 1) - theta_ref(i)) / dt;  
    }  
    w_ref(steps - 1) = 0;  

    // 初始状态  
    VectorXd X(7); // [x, y, theta, v, w] add xerr yerr 
    X << x_ref(0)-0.5, y_ref(0)+0.1, theta_ref(0), 0, 0,0,0;  
    std::vector<VectorXd> X_real;  
    X_real.push_back(X);  

    // 权重矩阵  
    Matrix3d Q;  
    Q << 1, 0, 0,  
         0, 1, 0,  
         0, 0, 5;  
    Matrix2d R;  
    R << 1, 0,  
         0, 1;  

    // 主循环  
    for (int k = 0; k < steps; ++k) {  
        int horizon = std::min(N, steps - k);  
        if (horizon == 0) break;  

        // 构建 Ak 和 Bk 矩阵  
        std::vector<Matrix3d> A(horizon);  
        std::vector<MatrixXd> B(horizon, MatrixXd::Zero(n, p));  
        for (int i = 0; i < horizon; ++i) {  
            A[i].setIdentity();  
            A[i](0, 2) = -v_ref(k + i) * sin(theta_ref(k + i)) * dt;  
            A[i](1, 2) = v_ref(k + i) * cos(theta_ref(k + i)) * dt;  

            B[i](0, 0) = cos(theta_ref(k + i)) * dt;  
            B[i](1, 0) = sin(theta_ref(k + i)) * dt;  
            B[i](2, 1) = dt;  
        }  

        // 构建 Ahat 和 Bhat 矩阵  
        MatrixXd Ahat = MatrixXd::Zero(n * horizon, n);  
        MatrixXd Bhat = MatrixXd::Zero(n * horizon, p * horizon);  
        Matrix3d Atemp = Matrix3d::Identity();  
        for (int i = 0; i < horizon; ++i) {  
            Atemp = A[i] * Atemp;  
            Ahat.block(n * i, 0, n, n) = Atemp;  

            MatrixXd Btemp = MatrixXd::Identity(n, n);  
            for (int j = i; j >= 0; --j) {  
                Bhat.block(n * i, p * j, n, p) = Btemp * B[j]; 
                
        //         std::cout << Bhat << std::endl;  
        // return 0;  
                Btemp = Btemp * A[j];  
            }  
        }  

        // std::cout << Bhat << std::endl;  
        // break;

        // 构建 Q_bar 和 R_bar  
        MatrixXd Q_bar = kroneckerProduct(MatrixXd::Identity(horizon, horizon), Q);  
        MatrixXd R_bar = kroneckerProduct(MatrixXd::Identity(horizon, horizon), R);  

        

        // 构造 Hessian 矩阵 H 和梯度 f  
        MatrixXd H = Bhat.transpose() * Q_bar * Bhat + R_bar;  
        H = 0.5 * (H + H.transpose()); // 确保对称性  
        VectorXd x0(3);  
        x0 << X(0) - x_ref(k), X(1) - y_ref(k), angle_bound(X(2) - theta_ref(k));  
        VectorXd f = 2 * (Ahat * x0).transpose() * Q_bar * Bhat;  

        // 转换为 OSQP 格式  
        SparseMatrix<double> H_sparse = H.sparseView();  
        VectorXd gradient = f;  
        SparseMatrix<double> A_constraint; // 约束矩阵（若存在）  
        VectorXd lowerBound, upperBound;  

        // 配置 OSQP  
        OsqpEigen::Solver solver;  
        solver.settings()->setVerbosity(false);  
        solver.data()->setNumberOfVariables(p * horizon);  
        solver.data()->setNumberOfConstraints(0); // 无不等式约束  
        solver.data()->setHessianMatrix(H_sparse);  
        solver.data()->setGradient(gradient);  

        if (!solver.initSolver()) {  
            std::cerr << "OSQP 初始化失败" << std::endl;  
            return -1;  
        }  

        // 求解  
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {  
            std::cerr << "OSQP 求解失败" << std::endl;  
            return -1;  
        }  

        // 获取控制量  
        VectorXd dertau = solver.getSolution();  
        double u1 = dertau(0) + v_ref(k);  
        double u2 = dertau(1) + w_ref(k);  


        // 更新状态  
        X(3) = u1 ;//+ 0.05 * (rand() % 100) / 100.0;  
        X(4) = u2 ;//+ 0.05 * (rand() % 100) / 100.0;  
        X(0) += X(3) * cos(X(2)) * dt;  
        X(1) += X(3) * sin(X(2)) * dt;  
        X(2) += X(4) * dt;  
        X(2) = angle_bound(X(2)); 
        if(k<steps-1) 
        {
        X(5) = X(0) - x_ref(k +1);
        X(6) = X(1) - y_ref(k +1);
        }
        else
        {
         X(5) = 0;
         X(6) = 0;       
        }
        X_real.push_back(X);  
    }  

 
    
    double t=0;
    //int i = 0;
    std::ofstream ofs("record_data.csv");
    ofs << "time,x,y,theta,v,w,e_x,e_y\n"; 
    for (auto& state : X_real) {
    	
    	std::cout << "x="<<state(0)<<" x_real.size = "<<X_real.size()<<std::endl;  
        ofs << t << "," 
            << state(0)<< "," 
            << state(1)<< "," 
            << state(2)  << "," 
            << state(3)  << "," 
            << state(4)  << "," 
            << state(5)  << ","                         
            << state(6)  << "\n"; 

        t = t+dt;

    }
    ofs.close();
    
    
    

    return 0;  
}  






