//
// Created by 15082 on 2026/3/12.
//

#ifndef APP_DOG_CTRL_H
#define APP_DOG_CTRL_H
#include "ctrl_pid.h"


namespace LegController {
    class wheel_speed_controller {
    public:
        wheel_speed_controller() = default;
         explicit wheel_speed_controller(const Controller::PID::pid_param_t &wheel_speed,
             const Controller::PID::pid_param_t &body_pos)
            :left_speed_pid(wheel_speed), right_speed_pid(wheel_speed)
            ,body_pos_pid(body_pos){
        }
        void speed_update(float target_forward,
            float left_current_speed, float right_current_speed,
            float body_current_pos, float body_current_gry);// forward:前进，单位m/s, spin:自转，单位rad/s
        void clear();
        void tick();
        [[nodiscard]] std::pair<float,float> output_get() const;
    private:
        Controller::PID left_speed_pid;
        Controller::PID right_speed_pid;
        Controller::PID body_pos_pid;
        float right_out_tor = 0, left_out_tor = 0;
        float body_current_pos_ = 0, left_current_speed_ = 0, right_current_speed_ = 0;
        float  forward_speed_ = 0, target_deg_ = 0;
    };
}



#endif //APP_DOG_CTRL_H
