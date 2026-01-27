//
// Created by 15082 on 2026/1/24.
//

#include "app_leg_ctrl.h"

void Controller::app_leg_ctrl::left_len_update(Relay::relay_leg left_leg, float target_len) {
    float32_t target_speed = left_length_pid_.update(left_leg.L0,target_len);
    output_.force_left = left_speed_pid_.update(left_leg.dot_L0,target_speed);
}
void Controller::app_leg_ctrl::right_len_update(Relay::relay_leg right_leg, float target_len) {
    float32_t target_speed = right_length_pid_.update(right_leg.L0,target_len);
    output_.force_right = right_speed_pid_.update(right_leg.dot_L0,target_speed);
}
void Controller::app_leg_ctrl::left_deg_update(Relay::relay_leg left_leg, float target_deg) {
    float32_t target_omega = left_deg_pid_.update(left_leg.theta,target_deg);
    output_.tor_left = left_omega_pid_.update(left_leg.dot_theta,target_omega);
}
void Controller::app_leg_ctrl::right_deg_update(Relay::relay_leg right_leg, float target_deg) {
    float32_t target_omega = right_deg_pid_.update(right_leg.theta,target_deg);
    output_.tor_right = right_omega_pid_.update(right_leg.dot_theta,target_omega);
}
void Controller::app_leg_ctrl::leg_clear() {
    left_speed_pid_.clear();
    right_speed_pid_.clear();
    left_deg_pid_.clear();
    right_deg_pid_.clear();
    left_length_pid_.clear();
    right_length_pid_.clear();
    left_omega_pid_.clear();
    right_omega_pid_.clear();
}