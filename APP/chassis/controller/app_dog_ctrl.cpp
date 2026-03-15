//
// Created by 15082 on 2026/3/12.
//

#include "app_dog_ctrl.h"
#include "const_data/robot_data.h"

#include <fast_math_functions.h>
using namespace LegController;

void wheel_speed_controller::speed_update(float forward, float body_deg,
    float left_current_speed, float right_current_speed, float body_current_pos) {
    forward_speed_ = forward;
    target_deg_ = body_deg;

    left_current_speed_ = left_current_speed;
    right_current_speed_ = right_current_speed;

    body_current_pos_ = body_current_pos;
}

void wheel_speed_controller::tick() {
    float target_speed_left, target_speed_right;
    float delta = body_current_pos_ - target_deg_;
    delta > PI? delta-= 2*PI:(delta < -PI? delta+= 2*PI:0);

    float target_gry = body_pos_pid.update(delta,0);
    target_speed_left = -target_gry*RL + forward_speed_;
    target_speed_right = target_gry*RL + forward_speed_;

    left_out_tor = left_speed_pid.update(left_current_speed_,target_speed_left);
    right_out_tor = right_speed_pid.update(right_current_speed_,target_speed_right);
}

void wheel_speed_controller::clear() {
    left_speed_pid.clear();
    right_speed_pid.clear();
    body_pos_pid.clear();
    forward_speed_ = 0;
    left_current_speed_ = right_current_speed_ = 0;
    left_out_tor = right_out_tor = 0;
}

std::pair<float, float> wheel_speed_controller::output_get() const {
    std::pair<float,float> output;
    output.first = left_out_tor;
    output.second = right_out_tor;
    return  output;
}
