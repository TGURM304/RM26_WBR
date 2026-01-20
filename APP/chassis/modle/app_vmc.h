//
// Created by 15082 on 2026/1/18.
//

#ifndef APP_VMC_H
#define APP_VMC_H
#include <matrix.h>

namespace VMC {
typedef enum {
    E_Left,
    E_Right
}E_LEG_SWITCH;
typedef struct {
    float p_left_tor1, p_left_tor2;
    float p_right_tor1, p_right_tor2;
    float c_left_tor1, c_left_tor2;
    float c_right_tor1, c_right_tor2;
}motor_tor;
typedef struct {
    float force_L, leg_tor; //输入的沿着腿的力和腿的扭矩
    float force_x, force_y; //输入的笛卡尔坐标系下的力
    float theta1, theta2, leg_theta, leg_len;
}update_pkg;
    class app_vmc {
    public:
        app_vmc() = default;
        void tor_clc(update_pkg pkg, E_LEG_SWITCH select);
        motor_tor tor_get() {return tor_;}
    private:
        void VMC_clc(float theta1, float theta2);
        float jacobin[4] = {};
        float p_force[2] = {};
        float c_force[2] = {};
        motor_tor tor_ = {0};
    };
}




#endif //APP_VMC_H
