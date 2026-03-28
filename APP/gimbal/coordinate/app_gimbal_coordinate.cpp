//
// Created by 15082 on 2026/3/28.
//

#include "app_gimbal_coordinate.h"

PID yaw_pos(25, 0, 0, 15, 0);
PID yaw_speed(2, 0.1 / 1000.0f, 0, 3, 0.2);

PID fric_speed(10, 1, 0, 16384, 3000);
PID trigger_speed(20, 0, 0, 16384, 10000);


using namespace Gimbal;

void gimbal_coordinate::tick() {
    auto cmd = ctrl_.get_cmd();
    auto flag = ctrl_.get_flag();

    if(flag->auto_aim_gimbal_ == true && flag->gimbal_ctrl_ == false) {
        head_->head_pid_clc(vision_->get_target().target_yaw, vision_->get_target().target_pitch);
    }

    shoot_->shoot_tick();
    head_->tick();
}

void gimbal_coordinate::init() {
    ctrl_.init();
    head_->head_init(yaw_pos,yaw_speed);
    shoot_->shoot_init(fric_speed,trigger_speed);
}

