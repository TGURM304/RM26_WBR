//
// Created by 15082 on 2026/1/24.
//

#ifndef APP_LEG_CTRL_H
#define APP_LEG_CTRL_H
#include "app_relay.h"
#include "ctrl_pid.h"

using namespace Controller;
namespace LegController {
    typedef struct {
        float32_t tor_left, tor_right;
        float32_t force_left, force_right;
    }leg_output;
    class app_leg_ctrl {
    public:
        app_leg_ctrl() = default;
        app_leg_ctrl(const PID::pid_param_t& speed_param,
                     const PID::pid_param_t& len_param,
                     const PID::pid_param_t& omega_param,
                     const PID::pid_param_t& deg_param)
            : left_speed_pid_(speed_param), right_speed_pid_(speed_param),
            left_length_pid_(len_param), right_length_pid_(len_param),
            left_omega_pid_(omega_param), right_omega_pid_(omega_param),
            left_deg_pid_(deg_param), right_deg_pid_(deg_param) {
        }
        void left_len_update(Relay::relay_leg left_leg, float target_len);
        void right_len_update(Relay::relay_leg right_leg, float target_len);
        void left_deg_update(Relay::relay_leg left_leg, float target_deg);
        void right_deg_update(Relay::relay_leg right_leg, float target_deg);
        leg_output get_output() {return output_;}
        void leg_clear();
    private:
        PID left_speed_pid_;
        PID right_speed_pid_;
        PID left_length_pid_;
        PID right_length_pid_;
        PID left_omega_pid_;
        PID right_omega_pid_;
        PID left_deg_pid_;
        PID right_deg_pid_;
        leg_output output_ = {0};
    };
}




#endif //APP_LEG_CTRL_H
