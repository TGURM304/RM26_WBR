//
// Created by 15082 on 2026/1/24.
//

#include "app_leg_ctrl.h"

void LegController::app_leg_ctrl::left_len_update(Relay::relay_leg left_leg, float target_len) {
    float32_t target_speed = left_length_pid_.update(left_leg.L0,target_len);
    output_.force_left = left_speed_pid_.update(left_leg.dot_L0,target_speed);
}

void LegController::app_leg_ctrl::right_len_update(Relay::relay_leg right_leg, float target_len) {
    float32_t target_speed = right_length_pid_.update(right_leg.L0,target_len);
    output_.force_right = right_speed_pid_.update(right_leg.dot_L0,target_speed);
}

void LegController::app_leg_ctrl::left_deg_update(Relay::relay_leg left_leg, float target_deg) {
    auto temp = target_deg - left_leg.theta;
    temp > PI? temp-= 2*PI:(temp < -PI? temp += PI: temp);
    float32_t target_omega = left_deg_pid_.update(0.0f,temp);
    output_.tor_left = left_omega_pid_.update(left_leg.dot_theta,target_omega);
}

void LegController::app_leg_ctrl::right_deg_update(Relay::relay_leg right_leg, float target_deg) {
    auto temp = target_deg - right_leg.theta;
    temp > PI? temp-= 2*PI:(temp < -PI? temp += PI: temp);
    float32_t target_omega = right_deg_pid_.update(0.0f,temp);
    output_.tor_right = right_omega_pid_.update(right_leg.dot_theta,target_omega);
}

void LegController::app_leg_ctrl::left_omega_only(Relay::relay_leg left_leg, float target_omega) {
    output_.tor_left = left_omega_pid_.update(left_leg.dot_theta,target_omega);
}

void LegController::app_leg_ctrl::right_omega_only(Relay::relay_leg right_leg, float target_omega) {
    output_.tor_right = right_omega_pid_.update(right_leg.dot_theta,target_omega);
}

void LegController::app_leg_ctrl::leg_clear() {
    left_speed_pid_.clear();
    right_speed_pid_.clear();
    left_deg_pid_.clear();
    right_deg_pid_.clear();
    left_length_pid_.clear();
    right_length_pid_.clear();
    left_omega_pid_.clear();
    right_omega_pid_.clear();
}

void LegController::app_leg_ctrl::leg_deg_clear() {
    left_omega_pid_.clear();
    left_deg_pid_.clear();
    right_omega_pid_.clear();
    right_deg_pid_.clear();
}

void LegController::app_leg_ctrl::left_leg_len_clear() {
    left_speed_pid_.clear();
    left_length_pid_.clear();
}

void LegController::app_leg_ctrl::right_leg_len_clear() {
     right_speed_pid_.clear();
    right_length_pid_.clear();
}

void LegController::app_leg_ctrl::left_omega_write_param(
    const double Kp, const double Ki, const double Kd, const double out_limit, const double iout_limit) {
    left_omega_pid_.set_para(Kp,Ki,Kd,out_limit,iout_limit);
}

void LegController::app_leg_ctrl::right_omega_write_param(
    const double Kp, const double Ki, const double Kd, const double out_limit, const double iout_limit) {
    right_omega_pid_.set_para(Kp,Ki,Kd,out_limit,iout_limit);
}

