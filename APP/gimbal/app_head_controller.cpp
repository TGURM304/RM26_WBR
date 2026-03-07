//
// Created by 15082 on 2026/2/4.
//

#include "app_head_controller.h"

#include "bsp_uart.h"

#include <fast_math_functions.h>

using namespace Gimbal;

void Head::head_init(const Controller::PID& yaw_pos_param,
        const Controller::PID& yaw_speed_param,
         const Controller::PID& pit_pos_param,
         const Controller::PID& pit_speed_param) {
    yaw_speed_ = yaw_speed_param;
    yaw_pos_ = yaw_pos_param;
    pit_speed_ = pit_speed_param;
    pit_pos_ = pit_pos_param;

    yaw_motor_->init();
    yaw_motor_->enable();
    pit_motor_->init();
    pit_motor_->reset();
    pit_motor_->enable();
}

void Head::head_pid_clc(float target_yaw, float target_pit) {
    head_update();
    float target_yaw_speed = yaw_pos_.update(gimbal_pkg_.ins_yaw,target_yaw*0.9f);
    // float target_yaw_speed = delta_yaw;
    float yaw_target_current = yaw_speed_.update(gimbal_pkg_.ins_yaw_dot,target_yaw_speed);
    // float yaw_target_current = yaw_speed_.update(gimbal_pkg_.yaw_speed,delta_yaw*10);

    //零点置位置0.04rad
    gimbal_pkg_.target_pit_motor_rad = 0.04f - target_pit;

    float yaw_out_temp = yaw_target_current*16384.0f/3.0f;
    pid_yaw_out_ = yaw_out_filter_.process(yaw_out_temp);
    float forward_temp = 0;
    forward_temp = 0.22f*tanhf(gimbal_pkg_.ins_yaw_dot*1.5f)*16384.0f/3.0f;
    pid_yaw_out_ += forward_temp;

    bsp_uart_printf(E_UART_DEBUG,"%f,%f,%f\r\n",
        gimbal_pkg_.target_pit_motor_rad, pit_motor_->status.pos,pit_motor_->status.torque);
    // pid_pit_out_ = pit_out;
}

void Head::head_relax() {
    yaw_motor_->update(0);
    pit_motor_->control(0,0,0,0,0);
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
    gimbal_pkg_.yaw_speed = yaw_motor_->status.speed/60.0f*2*PI;

    gimbal_pkg_.pit_pos = pit_motor_->status.pos;
    gimbal_pkg_.pit_speed = pit_motor_->status.vel;

    //这里是安装方向问题
    gimbal_pkg_.ins_pit = ins_->roll*PI/180.0f;
    gimbal_pkg_.ins_yaw = ins_->yaw*PI/180.0f;

    gimbal_pkg_.ins_pit_dot = ins_->raw.gyro[0];
    gimbal_pkg_.ins_yaw_dot = ins_->raw.gyro[2];
}

void Head::head_output(){
    yaw_motor_->update(pid_yaw_out_);
    // pit_motor_->control(0,0,0,0,pid_pit_out_);
    pit_motor_->control(gimbal_pkg_.target_pit_motor_rad,0,100,1.5,0);
}
