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

    // 拟合多项式 (二维):
    //   K_ij(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2
    //
    // K元素排列 (按行优先):
    //   n=0~9:   K[0][0~9] → T_{r→b} 对各状态的增益
    //   n=10~19: K[1][0~9] → T_{l→b} 对各状态的增益
    //   n=20~29: K[2][0~9] → T_{wr→r} 对各状态的增益
    //   n=30~39: K[3][0~9] → T_{wl→l} 对各状态的增益
    //
    // 系数顺序: [p00, p10, p01, p20, p11, p02]

    float32_t x0y0=1.0f, x1y0=left_leg.L0, x0y1 = right_leg.L0;
    float32_t x2y0 = left_leg.L0*left_leg.L0, x0y2 = right_leg.L0*right_leg.L0;
    float32_t x1y1 = left_leg.L0*right_leg.L0;
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j< 10; j++) {
            int idx = i*10 + j;
            dynamic_K_[i][j] = dynamic_coe_[idx][0]*x0y0 + dynamic_coe_[idx][1]*x1y0
                + dynamic_coe_[idx][2]*x0y1 + dynamic_coe_[idx][3]*x2y0
                + dynamic_coe_[idx][4]*x1y1 + dynamic_coe_[idx][5]*x0y2;
        }
    }

    dynamic_right_output_.body_move = -(
                                +delta_state[0]*dynamic_K_[0][0]+delta_state[1]*dynamic_K_[0][1]
                                +delta_state[2]*dynamic_K_[0][2]+delta_state[3]*dynamic_K_[0][3]);
    dynamic_right_output_.body_balance = -(
                            +delta_state[4]*dynamic_K_[0][4]+delta_state[5]*dynamic_K_[0][5]
                            +delta_state[6]*dynamic_K_[0][6]+delta_state[7]*dynamic_K_[0][7]
                            +delta_state[8]*dynamic_K_[0][8]+delta_state[9]*dynamic_K_[0][9]);
    dynamic_right_output_.wheel_move = -(
                                +delta_state[0]*dynamic_K_[2][0]+delta_state[1]*dynamic_K_[2][1]
                                +delta_state[2]*dynamic_K_[2][2]+delta_state[3]*dynamic_K_[2][3]);
    dynamic_right_output_.wheel_balance = -(
                            +delta_state[4]*dynamic_K_[2][4]+delta_state[5]*dynamic_K_[2][5]
                            +delta_state[6]*dynamic_K_[2][6]+delta_state[7]*dynamic_K_[2][7]
                            +delta_state[8]*dynamic_K_[2][8]+delta_state[9]*dynamic_K_[2][9]);

    dynamic_left_output_.body_move = -(
                                +delta_state[0]*dynamic_K_[1][0]+delta_state[1]*dynamic_K_[1][1]
                                +delta_state[2]*dynamic_K_[1][2]+delta_state[3]*dynamic_K_[1][3]);
    dynamic_left_output_.body_balance = -(
                            +delta_state[4]*dynamic_K_[1][4]+delta_state[5]*dynamic_K_[1][5]
                            +delta_state[6]*dynamic_K_[1][6]+delta_state[7]*dynamic_K_[1][7]
                            +delta_state[8]*dynamic_K_[1][8]+delta_state[9]*dynamic_K_[1][9]);
    dynamic_left_output_.wheel_move = -(
                                +delta_state[0]*dynamic_K_[3][0]+delta_state[1]*dynamic_K_[3][1]
                                +delta_state[2]*dynamic_K_[3][2]+delta_state[3]*dynamic_K_[3][3]);
    dynamic_left_output_.wheel_balance = -(
                            +delta_state[4]*dynamic_K_[3][4]+delta_state[5]*dynamic_K_[3][5]
                            +delta_state[6]*dynamic_K_[3][6]+delta_state[7]*dynamic_K_[3][7]
                            +delta_state[8]*dynamic_K_[3][8]+delta_state[9]*dynamic_K_[3][9]);

}
