//
// Created by 15082 on 2026/1/24.
//

#ifndef APP_RELAY_H
#define APP_RELAY_H
#include "app_observer.h"

namespace Relay {
    typedef struct {
        float S, dot_S;
        float phi, dot_phi;
        float left_theta, left_dot_theta;
        float right_theta, right_dot_theta;
        float body_theta, body_dot_theta;
        float data_x[10];
    }relay_lqr;
    typedef struct {
        float32_t L0, dot_L0;
        float32_t theta, dot_theta;
        float32_t theta_1, theta_2;//这里是等效腿上的夹角
    }relay_leg;
    typedef struct {
        float kalman_S, kalman_dot_S;
    }relay_kalman;
    typedef struct {
        float roll;
        float pitch;
        float yaw;
    }body_angle_;
    class message_adapter {
    public:
        message_adapter() = default;
        //todo:加入卡尔曼滤波
        explicit message_adapter(StateMapping* mapping):mapping_(mapping) {

        }
        void update();
        void clear();
        relay_lqr get_LQR_raw();
        relay_leg get_left_leg_status();
        relay_leg get_right_leg_status();

    private:
        StateMapping* mapping_ = nullptr;
        relay_lqr LQR_raw_data_ = {};
        relay_leg left_leg_status_ = {};
        relay_leg right_leg_status_ = {};
    };
}


#endif //APP_RELAY_H
