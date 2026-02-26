#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>
#include <fstream>

#define CTRL_FREQ 20 // Hz

struct record_data {
    double time;  

    double V;
    double omega;
    double V_ref;
    double omega_ref;

    double x;
    double y;
    double yaw;
    double x_ref;
    double y_ref;
    double yaw_ref;
};

struct way_point {
    double x;
    double y;
    double yaw;
    double forward_vel;
    double angular_vel;
};

struct pp_path {
    double x;
    double y;
};

class WayPoint {
private:
    way_point wp;
    std::vector<pp_path> ppps;
    double ld = 0.3;
    double acc = 0.2;
    double dec = 0.2;
    double max_vel = 0.5;
    double non_holonomic_dist = 0.5 * acc * std::pow(max_vel / acc, 2); // output: 1 m 
    bool is_success = false;
public:
    WayPoint(){
        ppps.push_back({0, 1});
        ppps.push_back({60, 1});
    };
    
    ~WayPoint(){};
    
    void get_target(const double now_pos[], const double now_forward_vel, double (&target)[]) {
        double add_x = 0;
        double m_x = 0;
        double m_y = 0;
        double m_yaw = 0;
        double temp_reasonable_dist = std::pow(ld, 2) - std::pow((now_pos[1] - ppps[0].y), 2);
        temp_reasonable_dist = std::fmax(temp_reasonable_dist, 0.0);
        if (temp_reasonable_dist > 0) {
            add_x = std::sqrt(temp_reasonable_dist);
        } else {
            add_x = 0;
        }
        m_x = now_pos[0] + add_x;
        m_y = ppps[0].y;
        if (m_x > ppps[1].x) {
            m_x = ppps[1].x;
        }

        double m_forward_vel = 0;
        double m_angular_vel = 0;
        double remain_dist = std::sqrt(std::pow(ppps[1].x - now_pos[0], 2) + std::pow(ppps[1].y - now_pos[1], 2));
        if (remain_dist < non_holonomic_dist) {
            m_forward_vel = now_forward_vel - dec * 1 / CTRL_FREQ;
        } else {
            if (now_forward_vel > this->max_vel) {
                m_forward_vel = this->max_vel;
            } else {
                m_forward_vel = now_forward_vel + acc * 1 / CTRL_FREQ;
            }
        }
        // if (remain_dist < 0.01 ) {
        if (remain_dist < 0.01 || now_pos[0] > ppps[1].x) {
            this->is_success = true;
        }

        target[0] = m_x;
        target[1] = m_y;
        target[3] = m_yaw;  
        target[3] = m_forward_vel;
        target[4] = m_angular_vel;
    };

    bool is_success_flag() {
        return this->is_success;
    };
};

class PPController
{
private:
    double m_alpha = 0;
    double m_ey_in_local_frame = 0;
    double angular_vel_max = 0.5;
    double angular_vel_min = -0.5;
    double temp_angular_vel = 0;
public:
    PPController(){};
    ~PPController(){};
    void get_cmd(double (&target)[5], const double forward_vel, const double (&now_pos)[3]){
        m_ey_in_local_frame = -(target[0] - now_pos[0]) * sin(now_pos[2]) + (target[1] - now_pos[1]) * cos(now_pos[2]);
        temp_angular_vel = 2 * forward_vel * m_ey_in_local_frame / (std::pow(target[0] - now_pos[0], 2) + std::pow(target[1] - now_pos[1], 2));
        // // target[4] = 2 * target[3] * m_ey_in_local_frame / (std::pow(target[0] - now_pos[0], 2) + std::pow(target[1] - now_pos[1], 2));

        // m_alpha = std::atan2(target[1] - now_pos[1], target[0] - now_pos[0]) - now_pos[2];
        // // double temp_angular_vel = 2 * forward_vel * std::sin(m_alpha) / std::sqrt((std::pow(target[0] - now_pos[0], 2) + std::pow(target[1] - now_pos[1], 2)));
        // temp_angular_vel = 2 * target[3] * std::sin(m_alpha) / std::sqrt((std::pow(target[0] - now_pos[0], 2) + std::pow(target[1] - now_pos[1], 2)));
        std::cout << "********** temp_angular_vel: " << temp_angular_vel << std::endl;
        
        if (temp_angular_vel > angular_vel_max) {
            target[4] = angular_vel_max;
        } else if (temp_angular_vel < angular_vel_min) {
            target[4] = angular_vel_min;
        } else {
            target[4] = temp_angular_vel;
        }
    };
};



int main() {
    std::vector<record_data> record_data_vec;

    WayPoint planner;
    PPController controller;
    double startPos[3] = { 0, 0.5, 1.0*M_PI/4 };
    int cmd_freq_ctrl = 0;
    int cmd_freq_ctrl_count = 100 / CTRL_FREQ;

    double t=0;
    double dt = 0.01;
    double nowPos[3] = { startPos[0] , startPos[1], startPos[2] };
    double V{0};
    double W{0};
    double PPref[5] = { 0 };

    while (!planner.is_success_flag())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        

        if ( cmd_freq_ctrl == 0 ) {
            planner.get_target(nowPos, V, PPref);
            controller.get_cmd(PPref, V, nowPos);
        }
        
        cmd_freq_ctrl++;
        cmd_freq_ctrl = cmd_freq_ctrl % cmd_freq_ctrl_count;
        
        V = PPref[3];
        W = PPref[4];

        nowPos[0] = nowPos[0] + dt * V * cos(nowPos[2]);
        nowPos[1] = nowPos[1] + dt * V * sin(nowPos[2]);
        nowPos[2] = nowPos[2] + dt * W;

        record_data_vec.push_back({t, V, W, PPref[3], PPref[4], nowPos[0], nowPos[1], nowPos[2], PPref[0], PPref[1], PPref[2]});
        std::cout << " V: " << V << " W: " << W << " V_ref: " << PPref[3] << " omega_ref: " << PPref[4] << "\n"
                    << " x: " << nowPos[0] << " y: " << nowPos[1] << "\n"
                    << " x_ref: " << PPref[0] << " y_ref: " << PPref[1] << std::endl;
        std::cout << "t: " << t << " Error X: " << nowPos[0] - PPref[0] << " Error Y: " << nowPos[1] - PPref[1]  << std::endl;
        t = t + dt;
    }

    std::ofstream ofs("record_data.csv");
    ofs << "time,V,omega,V_ref,omega_ref,x,y,yaw,x_ref,y_ref,yaw_ref\n"; 
    for (auto& data : record_data_vec) {
        ofs << data.time << "," 
            << data.V << "," 
            << data.omega << "," 
            << data.V_ref << ","
            << data.omega_ref << ","
            << data.x << "," 
            << data.y << "," 
            << data.yaw << "," 
            << data.x_ref << "," 
            << data.y_ref << "," 
            << data.yaw_ref << "\n"; 
    }
    ofs.close();

}
