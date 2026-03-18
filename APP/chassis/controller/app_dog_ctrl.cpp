//
// Created by 15082 on 2026/3/12.
//

#include "app_dog_ctrl.h"

#include "bsp_def.h"
#include "const_data/robot_data.h"

#include <fast_math_functions.h>
using namespace LegController;

void wheel_speed_controller::speed_update(float target_forward, float target_body_gry,
            float left_current_speed, float right_current_speed) {
    target_forward_speed_ = target_forward;
    target_body_gry_ = target_body_gry;

    left_current_speed_ = left_current_speed;
    right_current_speed_ = right_current_speed;
}

void wheel_speed_controller::tick() {
    float target_speed_left, target_speed_right;

    float target_spin_speed = target_body_gry_*RL;
    target_speed_left = -target_spin_speed + target_forward_speed_;
    target_speed_right = target_spin_speed + target_forward_speed_;

    left_out_tor = left_speed_pid.update(left_current_speed_,target_speed_left);
    right_out_tor = right_speed_pid.update(right_current_speed_,target_speed_right);
}

void wheel_speed_controller::clear() {
    left_speed_pid.clear();
    right_speed_pid.clear();
    target_forward_speed_ = 0;
    left_current_speed_ = right_current_speed_ = 0;
    left_out_tor = right_out_tor = 0;
}

std::pair<float, float> wheel_speed_controller::output_get() const {
    std::pair<float,float> output;
    output.first = left_out_tor;
    output.second = right_out_tor;
    return  output;
}
