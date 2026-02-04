//
// Created by 15082 on 2026/2/4.
//

#include "app_head.h"

#include <fast_math_functions.h>

using namespace Gimbal;

void Head::head_init(const Controller::PID& yaw_speed_param,
                     const Controller::PID& yaw_pos_param,
                     const Controller::PID& pit_speed_param,
                     const Controller::PID& pit_pos_param,
                     const Controller::PID& yaw_current_param) {
    yaw_speed_ = yaw_speed_param;
    yaw_pos_ = yaw_pos_param;
    yaw_current_ = yaw_current_param;
    pit_speed_ = pit_speed_param;
    pit_pos_ = pit_pos_param;

    yaw_motor_->init();
    yaw_motor_->enable();
    pit_motor_->init();
    pit_motor_->reset();
    pit_motor_->enable();
}

void Head::head_pid_clc(float delta_yaw, float delta_pit) {
    head_update();
    float target_yaw_speed = yaw_pos_.update(delta_yaw,0.0f);
    float yaw_target_tor = yaw_speed_.update(gimbal_pkg_.yaw_speed,target_yaw_speed);
    float yaw_target_current = GM6020_TOR_TO_CURRENT(yaw_target_tor);
    float yaw_out = yaw_current_.update(gimbal_pkg_.yaw_current,yaw_target_current);//计算得到的国际单位制电压

    float target_pit_speed = pit_pos_.update(delta_pit,0.0f);
    float pit_out = pit_speed_.update(pit_motor_->status.vel, target_pit_speed);

    pid_yaw_out_ = yaw_out;
    pid_pit_out_ = pit_out;
}

void Head::head_relax() {
    yaw_motor_->update(0);
    pit_motor_->update(0);
}

void Head::head_active(){
    yaw_motor_->enable();
    pit_motor_->reset();
    pit_motor_->enable();
}


void Head::head_update() {
    //更新yaw的内容到pkg中,均为国际单位制
    gimbal_pkg_.yaw_current = yaw_motor_->status.current*3.0f/16384.0f;
    float temp  =( yaw_motor_->status.angle - gimbal_pkg_.yaw_zero_point);
    if(temp > 8192/2.0f)
        temp -= 8192;
    else if(temp < -8192/2.0f)
        temp += 8192;
    gimbal_pkg_.yaw_pos = temp*2*PI/8192.0f;
    gimbal_pkg_.yaw_speed = yaw_motor_->status.speed*2*PI/60.0f;

    gimbal_pkg_.pit_pos = pit_motor_->status.pos;
    gimbal_pkg_.pit_speed = pit_motor_->status.vel;
}

void Head::head_output(){
    yaw_motor_->update(pid_yaw_out_*25000);
    pit_motor_->update(pid_pit_out_);
}
