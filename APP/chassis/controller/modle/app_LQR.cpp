//
// Created by 15082 on 2026/1/18.
//

#include "app_LQR.h"

using namespace LQR;

void LQR_controller::static_clc(float32_t *delta_state) {
    memcpy(delta_state,state_delta_,sizeof(float32_t)*10);
    Matrixf<10,1> delta_matrix(state_delta_);
    Matrixf<4,1> out_matrix = static_matrix*delta_matrix;
    for(int i = 0; i<4; i++) {
        out_tor[i] = out_matrix[i][0];
    }
}

void LQR_controller::dynamic_clc(float32_t *delta_state, leg_state_pkg leg_pkg_) {
    memcpy(delta_state,state_delta_,sizeof(float32_t)*10);
    float32_t x0y0=1.0f, x1y0=leg_pkg_.left_L0, x0y1 = leg_pkg_.right_L0;
    float32_t x2y0 = leg_pkg_.left_L0*leg_pkg_.left_L0, x0y2 = leg_pkg_.right_L0*leg_pkg_.right_L0;
    float32_t x1y1 = leg_pkg_.left_L0*leg_pkg_.right_L0;
    for(int i =0; i< 40; i++) {
        dynamic_K_[i] = dynamic_coe_[i*6]*x0y0 + dynamic_coe_[i*6+1]*x1y0
            + dynamic_coe_[i*6+2]*x0y1 + dynamic_coe_[i*6+3]*x2y0
            + dynamic_coe_[i*6+4]*x1y1 + dynamic_coe_[i*6+5]*x0y2;
    }
    Matrixf<4,10> dynamic_matrix(dynamic_K_);
    Matrixf<10,1> delta_matrix(state_delta_);
    Matrixf<4,1> out_matrix = dynamic_matrix*delta_matrix;
    for(int i = 0; i<4; i++)
        out_tor[i] = out_matrix[i][0];
}
