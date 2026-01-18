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
    float left_tor1, left_tor2;
    float right_tor1, right_tor2;
}motor_tor;
typedef struct {
    float force_L, leg_tor, leg_len;
    float theta_m1, theta_m2, leg_theta;
}update_pkg;
    class app_vmc {
    public:
        app_vmc();
        void force_clc(update_pkg pkg, E_LEG_SWITCH select);
    private:
        void VMC_clc(float theta_m1, float theta_m2);
        float jacobin[4] = {};
        float force[2];
        motor_tor tor_ = {0};
    };
}




#endif //APP_VMC_H
