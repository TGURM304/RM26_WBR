//
// Created by 15082 on 2026/1/18.
//

#include "app_LQR.h"

#include "lqr_matrix.h"

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

void LQR_controller::fit_clc(float32_t *delta_state, float left_len, float right_len) {
    float avr = (left_len + right_len) / 2.0f;
    float len_down = -1, len_up = -1;
    int cnt = 0;
    for(cnt = 0; cnt < 5; cnt++) {
        if(leg_len_arr[cnt] < avr)
            len_down = leg_len_arr[cnt];
        else if(leg_len_arr[cnt] >= avr) {
            len_up = leg_len_arr[cnt];
            break;
        }
        else if(cnt == 4 && leg_len_arr[cnt] < avr) {
            len_down = leg_len_arr[cnt];
            break;
        };
    }
    float temp_k[4][10] = {};
    //三种情况，在最大，在最小，在中间，分别进行处理
    if(len_down == -1) {
        memcpy(temp_k,K17,sizeof(K17));
    }
    else if(len_up == -1) {
        memcpy(temp_k,K34,sizeof(K34));
    }
    else if(len_down != len_up) {
        float percent = (avr - len_down) / (len_up - len_down);
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 10; j++) {
                temp_k[i][j] = K_ary[cnt][i][j]*(1.0f - percent) + K_ary[cnt-1][i][j]*percent;
            }
        }

    }
    // if(len_down != len_up && len_down != -1) {
    //     float percent = (avr - len_down) / (len_up - len_down);
    //     for(int i = 0; i < 4; i++) {
    //         for(int j = 0; j < 10; j++) {
    //             temp_k[i][j] = K_ary[cnt][i][j]*(1.0f - percent) + K_ary[cnt-1][i][j]*percent;
    //         }
    //     }
    // }
    // else if(cnt == 0)
    //     memcpy(temp_k,K17,sizeof(K17));
    // else if(cnt == 5)
    //     memcpy(temp_k,K34,sizeof(K34));

    right_output_.body_move = -(
                                +delta_state[0]*temp_k[0][0]+delta_state[1]*temp_k[0][1]
                                +delta_state[2]*temp_k[0][2]+delta_state[3]*temp_k[0][3]);
    right_output_.body_balance = -(
                            +delta_state[4]*temp_k[0][4]+delta_state[5]*temp_k[0][5]
                            +delta_state[6]*temp_k[0][6]+delta_state[7]*temp_k[0][7]
                            +delta_state[8]*temp_k[0][8]+delta_state[9]*temp_k[0][9]);
    right_output_.wheel_move = -(
                                +delta_state[0]*temp_k[2][0]+delta_state[1]*temp_k[2][1]
                                +delta_state[2]*temp_k[2][2]+delta_state[3]*temp_k[2][3]);
    right_output_.wheel_balance = -(
                            +delta_state[4]*temp_k[2][4]+delta_state[5]*temp_k[2][5]
                            +delta_state[6]*temp_k[2][6]+delta_state[7]*temp_k[2][7]
                            +delta_state[8]*temp_k[2][8]+delta_state[9]*temp_k[2][9]);

    left_output_.body_move = -(
                                +delta_state[0]*temp_k[1][0]+delta_state[1]*temp_k[1][1]
                                +delta_state[2]*temp_k[1][2]+delta_state[3]*temp_k[1][3]);
    left_output_.body_balance = -(
                            +delta_state[4]*temp_k[1][4]+delta_state[5]*temp_k[1][5]
                            +delta_state[6]*temp_k[1][6]+delta_state[7]*temp_k[1][7]
                            +delta_state[8]*temp_k[1][8]+delta_state[9]*temp_k[1][9]);
    left_output_.wheel_move = -(
                                +delta_state[0]*temp_k[3][0]+delta_state[1]*temp_k[3][1]
                                +delta_state[2]*temp_k[3][2]+delta_state[3]*temp_k[3][3]);
    left_output_.wheel_balance = -(
                            +delta_state[4]*temp_k[3][4]+delta_state[5]*temp_k[3][5]
                            +delta_state[6]*temp_k[3][6]+delta_state[7]*temp_k[3][7] + delta_state[8] * temp_k[3][8] + delta_state[9]*temp_k[3][9]);
}

void LQR_controller::soft_clc(float32_t *delta_state){
       right_output_.body_move = -(
                                +delta_state[0]*soft_K_[0][0]+delta_state[1]*soft_K_[0][1]
                                +delta_state[2]*soft_K_[0][2]+delta_state[3]*soft_K_[0][3]);
    right_output_.body_balance = -(
                            +delta_state[4]*soft_K_[0][4]+delta_state[5]*soft_K_[0][5]
                            +delta_state[6]*soft_K_[0][6]+delta_state[7]*soft_K_[0][7]
                            +delta_state[8]*soft_K_[0][8]+delta_state[9]*soft_K_[0][9]);
    right_output_.wheel_move = -(
                                +delta_state[0]*soft_K_[2][0]+delta_state[1]*soft_K_[2][1]
                                +delta_state[2]*soft_K_[2][2]+delta_state[3]*soft_K_[2][3]);
    right_output_.wheel_balance = -(
                            +delta_state[4]*soft_K_[2][4]+delta_state[5]*soft_K_[2][5]
                            +delta_state[6]*soft_K_[2][6]+delta_state[7]*soft_K_[2][7]
                            +delta_state[8]*soft_K_[2][8]+delta_state[9]*soft_K_[2][9]);

    left_output_.body_move = -(
                                +delta_state[0]*soft_K_[1][0]+delta_state[1]*soft_K_[1][1]
                                +delta_state[2]*soft_K_[1][2]+delta_state[3]*soft_K_[1][3]);
    left_output_.body_balance = -(
                            +delta_state[4]*soft_K_[1][4]+delta_state[5]*soft_K_[1][5]
                            +delta_state[6]*soft_K_[1][6]+delta_state[7]*soft_K_[1][7]
                            +delta_state[8]*soft_K_[1][8]+delta_state[9]*soft_K_[1][9]);
    left_output_.wheel_move = -(
                                +delta_state[0]*soft_K_[3][0]+delta_state[1]*soft_K_[3][1]
                                +delta_state[2]*soft_K_[3][2]+delta_state[3]*soft_K_[3][3]);
    left_output_.wheel_balance = -(
                            +delta_state[4]*soft_K_[3][4]+delta_state[5]*soft_K_[3][5]
                            +delta_state[6]*soft_K_[3][6]+delta_state[7]*soft_K_[3][7]
                            +delta_state[8]*soft_K_[3][8]+delta_state[9]*soft_K_[3][9]);

}
