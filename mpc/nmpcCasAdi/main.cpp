#include <vector>
#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <casadi/casadi.hpp>
 
using namespace std;
using namespace casadi;

const double dt = 0.01;
const double m_distanceOfWheel = 0.5;
const int mpc_horizon = 10;

struct record_data {
    double time;

    double x_ref;
    double y_ref;
    double yaw_ref;

    double x;
    double y;
    double yaw;

    double v;
    double w;

    double x_next;
    double y_next;
    double yaw_next;
};

void kinematics_update(double* pos, double vel, double angular_vel) {
    pos[0] = pos[0] + dt * vel * cos(pos[2]);
    pos[1] = pos[1] + dt * vel * sin(pos[2]);
    pos[2] = pos[2] + dt * angular_vel;
}

void kinematics_inverse(double inV, double inW, double& outVl, double& outVr) {
	outVl = 0.5 * (2* inV - inW * m_distanceOfWheel);
	outVr = 0.5 * (2* inV + inW * m_distanceOfWheel);
}

void kinematics_forword(double inVl, double inVr, double& outV, double& outW) {
	outV = 0.5 * (inVl + inVr);
	outW = (inVr - inVl) / m_distanceOfWheel;
}

void kinematics_horizon_update(const int iters_num, SX A, SX B, const SX xi, 
                                 const SX& du, SX& xi_horizon) {
  SX X = xi(std::vector<int> {0, 1, 2});
  SX u = xi(std::vector<int> {3, 4});
  for (int i = 0; i < iters_num; i++) {
    xi_horizon(std::vector<int> {5*i, 5*i+1, 5*i+2, 5*i+3, 5*i+4}) = vertcat(X, u);

    B(0, 0) = dt * cos(X(2));
    B(0, 1) = 0;
    B(1, 0) = dt * sin(X(2));
    B(1, 1) = 0;
    B(2, 0) = 0;
    B(2, 1) = dt;

    u = du(std::vector<int> {2*i, 2*i+1}) + u;
    X = mtimes(A, X) + mtimes(B,u);
  }
}

void controller_mpc(const double real_t, const double pos[3], const double u_last_frame[2], 
                        double u_mpc[2], double pos_ref[3]) {

  SX u_last = SX::vertcat({u_last_frame[0], u_last_frame[1]});
  SX du = SX::sym("du", 2 * mpc_horizon);
  SX xi_horizon = SX::zeros(5 * mpc_horizon, 1);
  SX W_horizon = SX::zeros(3 * mpc_horizon, 5);
  SX Z_horizon = SX::zeros(3 * mpc_horizon, 2 * mpc_horizon);
  SX Y_horizon = SX::zeros(3 * mpc_horizon, 1);

  SX A = SX::eye(3);
  SX B = SX::zeros(3, 2);
  SX X = SX::zeros(3, 1);

  SX C = horzcat(SX::eye(3), SX::zeros(3, 2));
  SX Y_error = SX::zeros((C.size(1)) * mpc_horizon, 1);

  SX Gain_Q = 1.0f * SX::eye(C.size(1) * mpc_horizon);
  for (int gain_index =0 ; gain_index < mpc_horizon; gain_index++) {
    Gain_Q(C.size(1)*gain_index, C.size(1) *gain_index) = 5.0f;
    Gain_Q(C.size(1)*gain_index+1, C.size(1)*gain_index+1) = 5.0f;
    Gain_Q(C.size(1)*gain_index+2, C.size(1)*gain_index+2) = 0.1f;
  }
  SX Gain_R = 0.0f * SX::eye(du.size(1));

  double Y_ref[3*mpc_horizon]{0};

  // state init.
  X(0) = pos[0];
  X(1) = pos[1];
  X(2) = pos[2];

  // A and B equ. init.
  B(0, 0) = dt * cos(X(2));
  B(0, 1) = 0;
  B(1, 0) = dt * sin(X(2));
  B(1, 1) = 0;
  B(2, 0) = 0;
  B(2, 1) = dt;
  for (int i = 0; i < mpc_horizon; i++) {
      Y_ref[3*i] = 3 * cos(M_PI/20 * (real_t + i * dt));
      Y_ref[3*i+1] = 3 * sin(M_PI/20 * (real_t + i * dt));
      Y_ref[3*i+2] = M_PI/2 + M_PI/20 * (real_t + i * dt);



    // if (real_t <= 39.7) {
    //   Y_ref[3*i] = real_t + i * dt;
    //   Y_ref[3*i+1] = real_t + i * dt;
    //   Y_ref[3*i+2] = M_PI/4;
    // } else {
    //   if ((real_t+i*dt)>=40) {
    //     Y_ref[3*i] = 40;
    //     Y_ref[3*i+1] = 40;
    //     Y_ref[3*i+2] = M_PI/4;
    //   } else {
    //     Y_ref[3*i] = real_t + i * dt;
    //     Y_ref[3*i+1] = real_t + i * dt;
    //     Y_ref[3*i+2] = M_PI/4;
    //   }
    // }
      
  }

  pos_ref[0] = Y_ref[0];
  pos_ref[1] = Y_ref[1];
  pos_ref[2] = Y_ref[2];

  // TODO: LOOP START
  SX xi = vertcat(X, u_last);

  kinematics_horizon_update(mpc_horizon, A, B, xi, du, xi_horizon);


  for (int i = 0; i < mpc_horizon; i++) {
    SX Ai_4_A = SX::eye(5);
    SX Ai_4_B = SX::eye(5);
    for (int j = i; j >= 0; j--) {
      A = SX::eye(3);
      B(0, 0) = dt * cos(xi_horizon(5*j+2));
      B(0, 1) = 0;
      B(1, 0) = dt * sin(xi_horizon(5*j+2));
      B(1, 1) = 0;
      B(2, 0) = 0;
      B(2, 1) = dt;

      SX A_bar_j= vertcat(horzcat(A, B),horzcat(SX::zeros(2,3), SX::eye(2)));
      SX B_bar_j= vertcat(B, SX::eye(2));
      if (j==i) {
        Ai_4_A = mtimes(A_bar_j, Ai_4_A);
      }

      // update Z_horizon
      SX CAiB_temp = mtimes(C, mtimes(Ai_4_B, B_bar_j));
      for (int Z_row_index = 0; Z_row_index < C.size(1); Z_row_index++) {
        for (int Z_col_index = 0; Z_col_index < B.size(2); Z_col_index++) {
          Z_horizon(i * C.size(1)+Z_row_index, j * B.size(2)+Z_col_index) = CAiB_temp(Z_row_index, Z_col_index);
        }
      }

      // compute Ai_4_B
      Ai_4_B = mtimes(Ai_4_B, A_bar_j);
    }

    // update W_horizon
    SX CAi_temp = mtimes(C, Ai_4_A);
    for (int W_row_index = 0; W_row_index < C.size(1); W_row_index++) {
      for (int W_col_index = 0; W_col_index < W_horizon.size(2); W_col_index++) {
        W_horizon(i * C.size(1)+W_row_index, W_col_index) = CAi_temp(W_row_index, W_col_index);
      }
    }

  }
  
  Y_horizon = mtimes(Z_horizon, du) + mtimes(W_horizon, xi);
  for (int i = 0; i < mpc_horizon; i++) {
    Y_error(i * C.size(1), 0) = Y_horizon(i * C.size(1), 0) - Y_ref[3*i];
    Y_error(i * C.size(1)+1, 0) = Y_horizon(i * C.size(1)+1, 0) - Y_ref[3*i+1];
    Y_error(i * C.size(1)+2, 0) = Y_horizon(i * C.size(1)+2, 0) - Y_ref[3*i+2];
  }

  // // method 1: not in the form of a quadratic program
  // SX f = mtimes(Y_error.T(), mtimes(Gain_Q, Y_error)) + mtimes(du.T(), mtimes(Gain_R, du));
  // // SX g = vertcat(x(0)*x(1)*x(2)*x(3), pow(x(0),2) + pow(x(1),2) + pow(x(2),2) + pow(x(3),2));

  // method 2: in the form of a quadratic program
  SX Y_ref_horizon = SX::zeros(3 * mpc_horizon, 1);
  for (int i = 0; i < mpc_horizon; i++) {
    Y_ref_horizon(i * C.size(1), 0) = Y_ref[3*i];
    Y_ref_horizon(i * C.size(1)+1, 0) = Y_ref[3*i+1];
    Y_ref_horizon(i * C.size(1)+2, 0) = Y_ref[3*i+2];
  }
  SX G_horizon = mtimes(W_horizon, xi) - Y_ref_horizon;
  SX f = mtimes(du.T(), mtimes((mtimes(Z_horizon.T(), mtimes(Gain_Q, Z_horizon)) + Gain_R), du)) + 
          2 * mtimes(G_horizon.T(), mtimes(Gain_Q, mtimes(Z_horizon, du)));
  // SX g = vertcat(x(0)*x(1)*x(2)*x(3), pow(x(0),2) + pow(x(1),2) + pow(x(2),2) + pow(x(3),2));
    
  // Initial guess
  std::vector<double> du0(2 * mpc_horizon, 0.0);
  // set bounds
  // vector<double> lbdu = { -10, 1, 1, 1 };
  // vector<double> ubdu = {5, 5, 5, 5 };

  // vector<double> lbg = { 25, 40 };
  // vector<double> ubg = { inf, 40 };

  // NLP
  SXDict nlp = { { "x", du }, { "f", f } };

  Dict opts;
  // opts["ipopt.hessian_approximation"] = "exact";
  // opts["ipopt.linear_solver"] = "mumps";
  // opts["ipopt.print_level"] = 3; 


  // Create NLP solver and buffers
  Function solver = nlpsol("solver", "ipopt", nlp, opts);
  std::map<std::string, DM> arg, res;

  // Solve the NLP
  // arg["lbx"] = lbdu;
  // arg["ubx"] = ubdu;
  // arg["lbg"] = lbg;
  // arg["ubg"] = ubg;
  arg["x0"] = du0;
  res = solver(arg);

  // // Print the solution
  // cout << "--------------------------------" << endl;
  // // std::cout << res << std::endl;
  // cout << "objective: " << res.at("f") << endl;
  // cout << "solution: " << res.at("x") << endl;

  
  u_mpc[0] = (double)res.at("x")(0);
  u_mpc[1] = (double)res.at("x")(1);

}


int main() {
  std::vector<record_data> record_data_vec;

  double real_t = 0.0;
  double pos[3] = {2, 0, 45 * M_PI / 180};
  double du_mpc[2] = {0, 0};
  double u_mpc[2] = {0, 0};
  double u_last_frame[2] = {0, 0};
  double pos_ref[3]{0};
  double last_pos[3]{0};

  for (int i = 0; i < 4000; i++) {
    cout << "i: " << i << endl;
    last_pos[0] = pos[0];
    last_pos[1] = pos[1];
    last_pos[2] = pos[2];
  
    controller_mpc(real_t, pos, u_last_frame, du_mpc, pos_ref);
    u_mpc[0] = u_last_frame[0]+du_mpc[0];
    u_mpc[1] = u_last_frame[1]+du_mpc[1];

    kinematics_update(pos, u_mpc[0], u_mpc[1]);

    record_data_vec.push_back({real_t, pos_ref[0], pos_ref[1], pos_ref[2]*180/M_PI, 
                              last_pos[0], last_pos[1], last_pos[2]*180/M_PI, 
                              u_mpc[0], u_mpc[1], pos[0], pos[1], pos[2]*180/M_PI});

    u_last_frame[0] = u_mpc[0];
    u_last_frame[1] = u_mpc[1];
    real_t += dt;
  }

  std::ofstream ofs("record_data.csv");
  ofs << "time,x_ref,y_ref,yaw_ref,x,y,yaw,v,w,x_next,y_next,yaw_next\n"; 
  for (auto& data : record_data_vec) {
        ofs << data.time << "," 
            << data.x_ref << "," 
            << data.y_ref << "," 
            << data.yaw_ref << ","
            << data.x << "," 
            << data.y << "," 
            << data.yaw << ","
            << data.v << ","
            << data.w << ","
            << data.x_next << ","
            << data.y_next << ","
            << data.yaw_next << "\n"; 
    }
    ofs.close();
 
  return 0;
}
