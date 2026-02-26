#include "common.hpp"

void kinematics_vel_2_rpm(double vel, double angular_vel, double* left, double* right) {
    *left = vel / m_radius - angular_vel * m_distance / (2 * m_radius);
    *right = vel / m_radius + angular_vel * m_distance / (2 * m_radius);
}

void kinematics_rpm_2_vel(double left, double right, double* vel, double* angular_vel) {
    *vel = (left + right) * m_radius / 2.0 ;
    *angular_vel = (right - left) * m_radius / m_distance ;
}
