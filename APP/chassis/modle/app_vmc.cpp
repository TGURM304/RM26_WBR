//
// Created by 15082 on 2026/1/18.
//

#include "app_vmc.h"

#include "cmath"

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
void app_vmc::VMC_clc(float theta_m1, float theta_m2) {
    
    jacobin[0] = -(L1*Sin(theta_m1)) - L2*(1 + (1 + (Power(L_a,2)*Sin(theta_m1 - theta_m2))/(Power(L_b,2)*Sqrt(1 - Power(-Power(L_a,2) + Power(L_b,2) + Power(L_a,2)*Cos(theta_m1 - theta_m2),2)/Power(L_b,4))))/2.)*Sin((3*theta_m1 - theta_m2 + ArcCos((-Power(L_a,2) + Power(L_b,2) + Power(L_a,2)*Cos(theta_m1 - theta_m2))/Power(L_b,2)))/2.);
    jacobin[1] = -0.5f*(L2*(-1 - (Power(L_a,2)*Sin(theta_m1 - theta_m2))/(Power(L_b,2)*Sqrt(1 - Power(-Power(L_a,2) + Power(L_b,2) + Power(L_a,2)*Cos(theta_m1 - theta_m2),2)/Power(L_b,4))))*Sin((3*theta_m1 - theta_m2 + ArcCos((-Power(L_a,2) + Power(L_b,2) + Power(L_a,2)*Cos(theta_m1 - theta_m2))/Power(L_b,2)))/2.f));
    jacobin[2] = L1*Cos(theta_m1) + L2*Cos((3*theta_m1 - theta_m2 + ArcCos((-Power(L_a,2) + Power(L_b,2) + Power(L_a,2)*Cos(theta_m1 - theta_m2))/Power(L_b,2)))/2.)*(1 + (1 + (Power(L_a,2)*Sin(theta_m1 - theta_m2))/(Power(L_b,2)*Sqrt(1 - Power(-Power(L_a,2) + Power(L_b,2) + Power(L_a,2)*Cos(theta_m1 - theta_m2),2)/Power(L_b,4))))/2.);
    jacobin[3] = (L2*Cos((3*theta_m1 - theta_m2 + ArcCos((-Power(L_a,2) + Power(L_b,2) + Power(L_a,2)*Cos(theta_m1 - theta_m2))/Power(L_b,2)))/2.)*(-1 - (Power(L_a,2)*Sin(theta_m1 - theta_m2))/(Power(L_b,2)*Sqrt(1 - Power(-Power(L_a,2) + Power(L_b,2) + Power(L_a,2)*Cos(theta_m1 - theta_m2),2)/Power(L_b,4)))))/2.;
}

void app_vmc::force_clc(update_pkg pkg, E_LEG_SWITCH select) {
    float *tor1 = &tor_.left_tor1;
    float *tor2 = &tor_.left_tor2;
    if(select == E_Right)
        tor1 = &tor_.right_tor1, tor2 = &tor_.right_tor2;
    VMC_clc(pkg.theta_m1,pkg.theta_m2);
    Matrixf<2,2> Jacobin_matrix(jacobin);
    Jacobin_matrix = Jacobin_matrix.trans();
    force[0] = -cos(pkg.leg_theta-PI/2)*pkg.leg_tor/pkg.leg_len-sin(pkg.leg_theta-PI/2)*pkg.force_L;
    force[1] = -sin(pkg.leg_theta-PI/2)*pkg.leg_tor/pkg.leg_len+cos(pkg.leg_theta-PI/2)*pkg.force_L;
    Matrixf<2,1> force_matrix(force);
    Matrixf<2,1> tor_matrix = Jacobin_matrix*force_matrix;
    *tor1 = tor_matrix[0][0];
    *tor2 = tor_matrix[0][1];
}

