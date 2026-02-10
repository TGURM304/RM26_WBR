//
// Created by 15082 on 2026/1/18.
//

#include "app_LQR.h"

using namespace LQR;

void LQR_controller::static_clc(float32_t *delta_state) {
    right_output_.body_move = -(
                                +delta_state[0]*static_K_[0][0]+delta_state[1]*static_K_[0][1]
                                +delta_state[2]*static_K_[0][2]+delta_state[3]*static_K_[0][3]);
    right_output_.body_balance = -(
                            +delta_state[4]*static_K_[0][4]+delta_state[5]*static_K_[0][5]
                            +delta_state[6]*static_K_[0][6]+delta_state[7]*static_K_[0][7]
                            +delta_state[8]*static_K_[0][8]+delta_state[9]*static_K_[0][9]);
    right_output_.wheel_move = -(
                                +delta_state[0]*static_K_[2][0]+delta_state[1]*static_K_[2][1]
                                +delta_state[2]*static_K_[2][2]+delta_state[3]*static_K_[2][3]);
    right_output_.wheel_balance = -(
                            +delta_state[4]*static_K_[2][4]+delta_state[5]*static_K_[2][5]
                            +delta_state[6]*static_K_[2][6]+delta_state[7]*static_K_[2][7]
                            +delta_state[8]*static_K_[2][8]+delta_state[9]*static_K_[2][9]);

    left_output_.body_move = -(
                                +delta_state[0]*static_K_[1][0]+delta_state[1]*static_K_[1][1]
                                +delta_state[2]*static_K_[1][2]+delta_state[3]*static_K_[1][3]);
    left_output_.body_balance = -(
                            +delta_state[4]*static_K_[1][4]+delta_state[5]*static_K_[1][5]
                            +delta_state[6]*static_K_[1][6]+delta_state[7]*static_K_[1][7]
                            +delta_state[8]*static_K_[1][8]+delta_state[9]*static_K_[1][9]);
    left_output_.wheel_move = -(
                                +delta_state[0]*static_K_[3][0]+delta_state[1]*static_K_[3][1]
                                +delta_state[2]*static_K_[3][2]+delta_state[3]*static_K_[3][3]);
    left_output_.wheel_balance = -(
                            +delta_state[4]*static_K_[3][4]+delta_state[5]*static_K_[3][5]
                            +delta_state[6]*static_K_[3][6]+delta_state[7]*static_K_[3][7]
                            +delta_state[8]*static_K_[3][8]+delta_state[9]*static_K_[3][9]);
}

void LQR_controller::dynamic_clc(float32_t *delta_state, Relay::relay_leg left_leg, Relay::relay_leg right_leg) {
    // memcpy(delta_state,state_delta_,sizeof(float32_t)*10);
    // float32_t x0y0=1.0f, x1y0=left_leg.L0, x0y1 = right_leg.L0;
    // float32_t x2y0 = left_leg.L0*left_leg.L0, x0y2 = right_leg.L0*right_leg.L0;
    // float32_t x1y1 = left_leg.L0*right_leg.L0;
    // for(int i =0; i< 40; i++) {
    //     dynamic_K_[i] = dynamic_coe_[i*6]*x0y0 + dynamic_coe_[i*6+1]*x1y0
    //         + dynamic_coe_[i*6+2]*x0y1 + dynamic_coe_[i*6+3]*x2y0
    //         + dynamic_coe_[i*6+4]*x1y1 + dynamic_coe_[i*6+5]*x0y2;
    // }
    // Matrixf<4,10> dynamic_matrix(dynamic_K_);
    // Matrixf<10,1> delta_matrix(state_delta_);
    // Matrixf<4,1> out_matrix = dynamic_matrix*delta_matrix;
    // for(int i = 0; i<4; i++)
    //     out_tor[i] = out_matrix[i][0];
}
