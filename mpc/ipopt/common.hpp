#ifndef COMMON_HPP
#define COMMON_HPP


#include <chrono>


#define CTRL_FREQ 5 // Hz= 100 / CTRL_FREQ
const double dt = 0.01;

const double m_distance = 0.35;
const double m_radius = 0.05;


void kinematics_vel_2_rpm(double vel, double angular_vel, double* left, double* right);

void kinematics_rpm_2_vel(double left, double right, double* vel, double* angular_vel);


#endif