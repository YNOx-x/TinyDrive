#include "controller.hpp"


struct record_data {
    double time;

    double x;
    double y;
    double yaw;
    double v;
    double w;
    double left_rpm;
    double right_rpm;

    double ref_x;
    double ref_y;
    double ref_yaw;
    double ref_v;
    double ref_w;
    double ref_left_rpm;
    double ref_right_rpm;
};

void kinematics_update(double* pos, double vel, double angular_vel) {
    pos[0] = pos[0] + dt * vel * cos(pos[2]);
    pos[1] = pos[1] + dt * vel * sin(pos[2]);
    pos[2] = pos[2] + dt * angular_vel;
}

void get_current_ref_path(const double real_t, double ref_info[]) {
    // circle
    ref_info[0] = 3 * cos(M_PI/20 * real_t);
    ref_info[1] = 3 * sin(M_PI/20 * real_t);
    ref_info[2] = M_PI/2 + M_PI/20 * real_t;
    ref_info[3] = 3*M_PI/20;
    ref_info[4] = M_PI/20;
    kinematics_vel_2_rpm(ref_info[3], ref_info[4], &ref_info[5], &ref_info[6]);

    // // line
    // ref_info[0] = real_t;
    // ref_info[1] = real_t;
    // ref_info[2] = M_PI/4;
    // ref_info[3] = sqrt(2.0);
    // ref_info[4] = 0.0;
    // kinematics_vel_2_rpm(ref_info[3], ref_info[4], &ref_info[5], &ref_info[6]);

    // // curve
    // double dxref = 1.0;
    // double ddxref = 0;
    // double dyref = -3 * M_PI/20 * sin(M_PI/20 * real_t);
    // double ddyref = -3 * M_PI * M_PI / (20 * 20) * cos(M_PI/20 * real_t);
    // ref_info[0] = real_t;
    // ref_info[1] = 3 * cos(M_PI/20 * real_t);
    // ref_info[2] = atan2(dyref, dxref);
    // ref_info[3] = sqrt(dxref*dxref + dyref*dyref);
    // ref_info[4] = (dxref * ddyref - dyref * ddxref) / (dxref * dxref + dyref * dyref);
    // kinematics_vel_2_rpm(ref_info[3], ref_info[4], &ref_info[5], &ref_info[6]);
}


int main()
{
    std::vector<record_data> record_data_vec;
    int ctrl_index = 0;
    double real_t = 0.0;
    double pos[3] = {2.5, 0.2, 45 * M_PI / 180};
    double u_mpc[2] = {0, 0};
    double ref_info[7] = {0, 0, 0, 0, 0, 0, 0}; // x,y,yaw,v,w,left_rpm,right_rpm
    int time_total = 20000;


    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
    app->Options()->SetStringValue("hessian_approximation", "exact");
    app->Options()->SetIntegerValue("print_level", 0);
    app->Options()->SetStringValue("print_timing_statistics", "no");
    app->Options()->SetIntegerValue("max_iter", 500);
    app->Options()->SetNumericValue("tol", 1e-9); // match inf_du in out info.
    app->Options()->SetStringValue("mu_strategy", "adaptive");

    ApplicationReturnStatus status;
    status = app->Initialize();
    if (status != Solve_Succeeded) {
        std::cout << "IPOPT初始化失败" << std::endl;
        return -1;
    }
    SmartPtr<MpcController> mpc_nlp = new MpcController(real_t, pos);

    for (int i = 0; i < time_total; i++) {
        get_current_ref_path(real_t, ref_info);

        if (ctrl_index == 0) {
            mpc_nlp->update_problem(real_t, pos);
            status = app->OptimizeTNLP(mpc_nlp);

            // //L: debug
            // if (status == Solve_Succeeded || status == Solved_To_Acceptable_Level) {
            //     std::cout << "优化成功" << std::endl;
            // } else {
            //     std::cout << "优化失败" << std::endl;
            // }
            
            u_mpc[0] = mpc_nlp->o_mpc[0];
            u_mpc[1] = mpc_nlp->o_mpc[1];
        }
std::cout << "time: " << real_t << " u_mpc:" << u_mpc[0] << " , " << u_mpc[1] << std::endl;
        kinematics_update(pos, u_mpc[0], u_mpc[1]);
        double left_rpm, right_rpm;
        kinematics_vel_2_rpm(u_mpc[0], u_mpc[1], &left_rpm, &right_rpm);
        record_data_vec.push_back({real_t, pos[0], pos[1], pos[2]*180/M_PI, u_mpc[0], u_mpc[1], left_rpm, right_rpm,
                ref_info[0], ref_info[1], ref_info[2]*180/M_PI, ref_info[3], ref_info[4], ref_info[5], ref_info[6]});
        real_t += dt;

        ctrl_index = (ctrl_index + 1) % CTRL_FREQ;        
    }

    std::ofstream ofs("record_data.csv");
    ofs << "time,x,y,yaw,v,w,left_rpm,right_rpm,ref_x,ref_y,ref_yaw,ref_v,ref_w,ref_left_rpm,ref_right_rpm\n"; 
    for (auto& data : record_data_vec) {
        ofs << data.time << "," 
            << data.x << "," 
            << data.y << "," 
            << data.yaw << "," 
            << data.v << "," 
            << data.w << ","
            << data.left_rpm << ","
            << data.right_rpm << ","
            << data.ref_x << ","
            << data.ref_y << ","
            << data.ref_yaw << ","
            << data.ref_v << ","
            << data.ref_w << ","
            << data.ref_left_rpm << ","
            << data.ref_right_rpm << "\n"; 
    }
    ofs.close();

    return 0;
}
