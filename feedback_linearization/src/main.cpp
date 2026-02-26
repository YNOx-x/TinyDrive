#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>
#include <fstream>


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

int main()
{
    std::vector<record_data> record_data_vec;

    double startPos[3] = { -2, -2, M_PI/4 };

    int cmd_freq_ctrl = 0;
    int cmd_freq_ctrl_count = 35;

    double t=0;
    double dt = 0.01;
    double nowPos[3] = { startPos[0] , startPos[1], startPos[2] };
    double refPos[3] = { 0 };
    double refDPos[3] = { 0 };
    double refDDPos[3] = { 0 };
    double refVel = { 0 };
    double refW = { 0 };


    double kpx=1;
    double kpy=1;
    double kptheta=1;

    double r1{0};
    double r2{0};
    double r3{0};

    double V{0};
    double W{0};

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        t = t + dt;

        refPos[0] = t;
        refPos[1] = t;
        refPos[2] = M_PI/4;

        refDPos[0] = 1;
        refDPos[1] = 1;
        refDPos[2] = 0;

        refDDPos[0] = 0;
        refDDPos[1] = 0;
        refDDPos[2] = 0;

        refVel = sqrt(refDPos[0] * refDPos[0] + refDPos[1] * refDPos[1]);
        if (fabs(refVel) > 1e-3){
            refW = (refDPos[0] * refDDPos[1] - refDPos[1] * refDDPos[0]) / (refDPos[0] * refDPos[0] + refDPos[1] * refDPos[1]);
        }else{
            refW = 0;
        }


        if ( cmd_freq_ctrl == 0 ) {
            r1 = refDPos[0] + kpx * (refPos[0] - nowPos[0]);
            r2 = refDPos[1] + kpy * (refPos[1] - nowPos[1]);
            r3 = refDPos[2] + kptheta * (refPos[2] - nowPos[2]);

            V = (r1*cos(nowPos[2]))+(r2*sin(nowPos[2])) ;
            W = r3 ;

            // V = (r1*cos(nowPos[2]))+(r2*sin(nowPos[2])) + 0.2*((double)(rand() % 100)/100 - 0.5);
            // W = r3 + 0.5*((double)(rand() % 100)/100 - 0.5);

            // V = (r1*cos(refPos[2]))+(r2*sin(refPos[2])) ;
            // W = r3 ;
            
        }
        
        cmd_freq_ctrl++;
        cmd_freq_ctrl = cmd_freq_ctrl % cmd_freq_ctrl_count;


        nowPos[0] = nowPos[0] + dt * V * cos(nowPos[2]);
        nowPos[1] = nowPos[1] + dt * V * sin(nowPos[2]);
        nowPos[2] = nowPos[2] + dt * W;

        record_data_vec.push_back({t, V, W, refVel, refW, nowPos[0], nowPos[1], nowPos[2], refPos[0], refPos[1], refPos[2]});

        std::cout << "t: " << t << " Error X: " << nowPos[0] - refPos[0] << " Error Y: " << nowPos[1] - refPos[1]  << std::endl;
        if (t > 200) break;
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
