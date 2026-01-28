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
    float32_t* get_out_tor() {
        return out_tor;
    }
private:
    float32_t static_K_[40]{};
    float32_t dynamic_coe_[240]{};
    float32_t dynamic_K_[40] = {0.0f};
    float32_t state_delta_[10] = {0.0f};
    float32_t out_tor[4] = {0.0f};
};
}

#endif //APP_LQR_H
