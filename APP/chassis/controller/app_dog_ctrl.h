//
// Created by 15082 on 2026/3/12.
//

#ifndef APP_DOG_CTRL_H
#define APP_DOG_CTRL_H
#include "ctrl_pid.h"


namespace LegController {
    class wheel_speed_controller {
        //控制机体旋转角速度和前进速度
    public:
        wheel_speed_controller() = default;
        explicit wheel_speed_controller(const Controller::PID::pid_param_t &wheel_speed)
            :left_speed_pid(wheel_speed), right_speed_pid(wheel_speed) {
        }
        void speed_update(float target_forward, float target_body_gry,
            float left_current_speed, float right_current_speed);// forward:前进，单位m/s, spin:自转，单位rad/s
        void clear();
        void tick();
        [[nodiscard]] std::pair<float,float> output_get() const;
    private:
        Controller::PID left_speed_pid;
        Controller::PID right_speed_pid;
        float right_out_tor = 0, left_out_tor = 0;
        float left_current_speed_ = 0, right_current_speed_ = 0;
        float target_forward_speed_ = 0, target_body_gry_ = 0;
    };
}



#endif //APP_DOG_CTRL_H
