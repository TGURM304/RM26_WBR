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
    pit_pos_ = pit_pos_param;
    pit_speed_ = pit_speed_param;

    yaw_motor_->init();
    yaw_motor_->enable();
    pit_motor_->init();
    pit_motor_->reset();
    pit_motor_->enable();
}

void Head::head_clear() {
    yaw_speed_.clear();
    yaw_pos_.clear();
    yaw_out_filter_.clear();
    head_flag_ = 0;
}

void Head::head_pid_clc(float target_yaw, float target_pit) {
    head_update();
    float yaw_delta = robo_snap_->get_snap_pkg().ins_yaw -target_yaw;
    yaw_delta > PI ? yaw_delta -= 2*PI : (yaw_delta < -PI ? yaw_delta += 2*PI : 0);
    float target_yaw_speed = yaw_pos_.update(yaw_delta,0);
    float yaw_target_current = yaw_speed_.update(robo_snap_->get_snap_pkg().ins_yaw_dot,target_yaw_speed);

    float yaw_out_temp = yaw_target_current*16384.0f/3.0f;
    pid_yaw_out_ = yaw_out_filter_.process(yaw_out_temp);
    float forward_temp = 0;
    forward_temp = 0.22f*tanhf(robo_snap_->get_snap_pkg().ins_yaw_dot*1.5f)*16384.0f/3.0f;
    pid_yaw_out_ += forward_temp;

    //此处默认轮腿身子是平的了，后续可以增加姿态补偿

    float target_ver = pit_pos_.update(robo_snap_->get_snap_pkg().ins_pit,target_pit);
    float pid = pit_speed_.update(robo_snap_->get_snap_pkg().ins_pit_dot,target_ver);
    float pit_forward = 0.3f*tanhf(robo_snap_->get_snap_pkg().pit_motor_ver*2.3f);
    pit_forward += 0.7;
    pid_pit_out_ = pid + pit_forward;
    // pid_pit_out_ = pit_out_filter_.process(pid_pit_out_);
    app_msg_vofa_send(E_UART_1,
        robo_snap_->get_snap_pkg().ins_pit,
        target_pit,
        robo_snap_->get_snap_pkg().ins_pit_dot,
        target_ver,
        pid,
        pit_forward);

}

void Head::head_relax() {
    yaw_motor_->update(0);
    pit_motor_->control(0,0,0,0,0);
    pid_yaw_out_ = 0, pid_pit_out_ = 0;
}

void Head::head_active(){
    yaw_motor_->enable();
    pit_motor_->reset();
    pit_motor_->enable();
}


void Head::head_update() {
    //更新yaw的内容到pkg中,均为国际单位制
    robo_snap_->snap_update();
}

void Head::head_output(){
    yaw_motor_->update(pid_yaw_out_);
    pit_motor_->control(head_ctrl_pkg_.pit_pos,0,0,0,pid_pit_out_);
}
