//
// Created by 15082 on 2026/1/18.
//

#include "app_vmc.h"

#include "app_msg.h"
#include "cmath"

#include <app_observer.h>

#define Power std::pow
#define Sin std::sin
#define Cos std::cos
#define ArcCos std::acos
#define Sqrt std::sqrt
#define L1 0.210f
#define L2 0.250f
#define L_a 0.0945f
#define L_b 0.1125f

using namespace VMC;
void app_vmc::VMC_clc(float theta1, float theta2) {
    //此处直接计算了转置后的矩阵
    jacobin[0] = -L1*sin(theta1)-L2*sin(theta1+theta2);
    jacobin[2] = -L2*sin(theta1+theta2);
    jacobin[1] = L1*cos(theta1)+L2*cos(theta1+theta2);
    jacobin[3] = L2*cos(theta1+theta2);
}

void app_vmc::tor_clc(ctrl_pkg pkg, Relay::relay_leg status,E_LEG_SWITCH select) {
    float *p_tor1 = &tor_.p_left_tor1, *c_tor1 = &tor_.c_left_tor1;
    float *p_tor2 = &tor_.p_left_tor2, *c_tor2 = &tor_.c_left_tor2;
    if(select == E_Right) {
        p_tor1 = &tor_.p_right_tor1, c_tor1 = &tor_.c_right_tor1;
        p_tor2 = &tor_.p_right_tor2, c_tor2 = &tor_.c_right_tor2;
    }
    VMC_clc(status.theta_1,status.theta_2);
    //此处直接计算了转置后的矩阵
    Matrixf<2,2> Jacobin_matrix(jacobin);
    auto force_tor = pkg.leg_tor/status.L0;
    auto magic_x = pkg.force_L*std::cos(status.theta) + force_tor*std::cos(status.theta + PI/2);
    auto magic_y = pkg.force_L*std::sin(status.theta) + force_tor*std::sin(status.theta + PI/2);

    p_force[0] = magic_x;
    p_force[1] = magic_y;
    c_force[0] = pkg.force_x;
    c_force[1] = pkg.force_y;

    Matrixf<2,1> p_force_matrix(p_force);
    Matrixf<2,1> c_force_matrix(c_force);
    Matrixf<2,1> tor_matrix = Jacobin_matrix*p_force_matrix;
    Matrixf<2,1> cor_matrix = Jacobin_matrix*c_force_matrix;
    *p_tor1 = tor_matrix[0][0];
    *p_tor2 = tor_matrix[1][0];
    *c_tor1 = cor_matrix[0][0];
    *c_tor2 = cor_matrix[1][0];
}
