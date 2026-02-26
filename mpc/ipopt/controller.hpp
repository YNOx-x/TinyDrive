#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <iostream>
#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <eigen3/Eigen/Sparse>
#include <fstream>
#include <vector>

#include "common.hpp"


using namespace Ipopt;

class MpcController : public TNLP 
{
public:
    typedef struct __RefInfoData{
        double x;
        double y;
        double yaw;
        double v;
        double w;
        double left_rpm;
        double right_rpm;
    }RefInfoData;

    MpcController(const double real_t, const double pos[]);

    void update_problem(const double real_t, const double pos[]);

    void cal_ref_info(const double real_t, std::vector<RefInfoData>& ref_info_vec);

    virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                              Index& nnz_h_lag, IndexStyleEnum& index_style) ;

    virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u,
                                 Index m, Number* g_l, Number* g_u) ;

    virtual bool get_starting_point(Index n, bool init_x, Number* x,
                                    bool init_z, Number* z_L, Number* z_U,
                                    Index m, bool init_lambda, Number* lambda) ;

    virtual bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value) ;

    virtual bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f) ;

    virtual bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g) ;

    virtual bool eval_jac_g(Index n, const Number* x, bool new_x,
                            Index m, Index nele_jac, Index* iRow, Index *jCol,
                            Number* values) ;

    // 海森矩阵结构
    virtual bool eval_h(Index n, const Number* x, bool new_x,
                        Number obj_factor, Index m, const Number* lambda,
                        bool new_lambda, Index nele_hess, Index* iRow,
                        Index* jCol, Number* values) ;

    virtual void finalize_solution(SolverReturn status,
                                   Index n, const Number* x, const Number* z_L, const Number* z_U,
                                   Index m, const Number* g, const Number* lambda,
                                   Number obj_value,
                                   const IpoptData* ip_data,
                                   IpoptCalculatedQuantities* ip_cq) ;
    
    // void kinematics_vel_2_rpm(double vel, double angular_vel, double* left, double* right);
    // void kinematics_rpm_2_vel(double left, double right, double* vel, double* angular_vel);

    double o_mpc[2];
private:
    double i_real_t;
    double i_pos[3];

    const int mpc_horizon = 30;
    const double m_vel_diff = 2.0;
    const double m_wheels_vel_max = 32; // 2.0e19;; // 

    
    std::vector<RefInfoData> ref_info_vec;  // mpc_horizon =30

    Eigen::VectorXd pos_error = Eigen::VectorXd::Zero(3);
    
    Eigen::MatrixXd gain_q = 1.0 * Eigen::MatrixXd::Identity(3*mpc_horizon, 3*mpc_horizon);
    Eigen::MatrixXd gain_r = 1.0 * Eigen::MatrixXd::Identity(2*mpc_horizon, 2*mpc_horizon);

    Eigen::MatrixXd A_bar = Eigen::MatrixXd::Zero(3*mpc_horizon, 3);
    Eigen::MatrixXd B_bar = Eigen::MatrixXd::Zero(3*mpc_horizon, 2*mpc_horizon);

    
    Eigen::MatrixXd target_quad_iteam;
    Eigen::MatrixXd target_linear_iteam;
};

#endif
