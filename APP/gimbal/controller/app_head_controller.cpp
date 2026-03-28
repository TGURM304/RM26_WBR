//
// Created by 15082 on 2026/2/4.
//

#include "app_head_controller.h"

#include "bsp_uart.h"

#include <fast_math_functions.h>

using namespace Gimbal;

void Head::head_init(const Controller::PID& yaw_pos_param,
    const Controller::PID& yaw_speed_param) {
    yaw_speed_ = yaw_speed_param;
    yaw_pos_ = yaw_pos_param;

    yaw_motor_->init();
    yaw_motor_->enable();
    pit_motor_->init();
    pit_motor_->reset();
    pit_motor_->enable();
}

void Head::head_clear() {
    head_active_flag = false;
    yaw_speed_.clear();
    yaw_pos_.clear();
    yaw_out_filter_.clear();
    head_flag_ = 0;
}

void Head::head_pid_clc(float target_yaw, float target_pit) {
    head_active_flag = true;
    head_update();
    float yaw_delta = robo_snap_->get_snap_pkg().ins_yaw -target_yaw;
    yaw_delta > PI ? yaw_delta -= 2*PI : (yaw_delta < -PI ? yaw_delta += 2*PI : 0);
    float target_yaw_speed = yaw_pos_.update(yaw_delta,0);
    float yaw_target_current = yaw_speed_.update(robo_snap_->get_snap_pkg().ins_yaw_dot,target_yaw_speed);

    //此处默认轮腿身子是平的了，后续可以增加姿态补偿
    float pit_target = robo_snap_->get_pitch_zero() + target_pit;
    pit_target > PIT_LIMIT_MAX ? pit_target = PIT_LIMIT_MAX :
        (pit_target < PIT_LIMIT_MIN ? pit_target = PIT_LIMIT_MIN : 0);
    head_ctrl_pkg_.pit_pos = pit_target;

    float yaw_out_temp = yaw_target_current*16384.0f/3.0f;
    pid_yaw_out_ = yaw_out_filter_.process(yaw_out_temp);
    float forward_temp = 0;
    forward_temp = 0.22f*tanhf(robo_snap_->get_snap_pkg().ins_yaw_dot*1.5f)*16384.0f/3.0f;
    pid_yaw_out_ += forward_temp;
}

void Head::head_relax() {
    head_active_flag = false;
    pid_yaw_out_ = 0;
}

void Head::head_active(){
    yaw_motor_->enable();
    pit_motor_->reset();
    pit_motor_->enable();
    head_active_flag = true;
}


void Head::head_update() {
    //更新yaw的内容到pkg中,均为国际单位制
    robo_snap_->snap_update();
}

void Head::tick(){
    if(head_active_flag) {
        yaw_motor_->update(pid_yaw_out_);
        pit_motor_->control(head_ctrl_pkg_.pit_pos,0,40,2,0);
    }
    else {
        yaw_motor_->update(0);
        pit_motor_->control(0,0,0,0,0);
    }
}
