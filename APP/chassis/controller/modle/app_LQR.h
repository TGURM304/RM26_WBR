//
// Created by 15082 on 2026/1/18.
//

#ifndef APP_LQR_H
#define APP_LQR_H

#include "app_observer.h"
#include "app_vmc.h"


namespace LQR {

typedef struct {
    float32_t left_L0, left_theta;
    float32_t right_L0, right_theta;
}leg_state_pkg;
typedef struct {
    float32_t body_balance, body_move;
    float32_t wheel_balance, wheel_move;
}lqr_output;
typedef enum {
    E_left,
    E_right
}leg_switch;
class LQR_controller {
    /*
     * 设计LQR控制器
     * 我们希望他只作为一个运算黑箱存在
     * 输入delta，运算选项，输出torque
     */
public:
    LQR_controller(float32_t *static_K, float32_t *dynamic_coe) {
        memcpy(static_K_,static_K,sizeof(float32_t)*40);
        memcpy(dynamic_coe_,dynamic_coe,sizeof(float32_t)*240);
    }
    void static_clc(float32_t *delta_state);
    void dynamic_clc(float32_t *delta_state, Relay::relay_leg left_leg, Relay::relay_leg right_leg);
    lqr_output get_lqr_output(leg_switch leg) {
        if(leg == E_left) {return left_output_;}
        else { return right_output_;}
    }
    lqr_output get_dynamic(leg_switch leg) {
        if(leg == E_left) {return dynamic_left_output_;}
        else { return dynamic_right_output_;}
    }
private:
    float32_t static_K_[4][10] = {};
    float32_t dynamic_coe_[40][6] = {};
    float32_t dynamic_K_[4][10] = {0.0f};
    float32_t state_delta_[10] = {0.0f};

    lqr_output left_output_ = {};
    lqr_output right_output_ = {};

    lqr_output dynamic_left_output_ = {};
    lqr_output dynamic_right_output_ = {};
};
}

#endif //APP_LQR_H
