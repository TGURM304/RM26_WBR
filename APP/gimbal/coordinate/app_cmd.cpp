//
// Created by 15082 on 2026/3/28.
//

#include "app_cmd.h"
using namespace Gimbal_cmd;

void gimbal_ctrl::init() {
    gimbal_pkg_.chassis_cmd_ = Coordinate::mode_state::E_WAITING;
    gimbal_pkg_.switch_cmd_ = Coordinate::mode_switch_cmd::CMD_WAITING;
    gimbal_pkg_.delta_yaw_ = 0;
    gimbal_pkg_.dot_S = 0;

    chassis_pkg_.chassis_cmd_ = Coordinate::E_WAITING;
}

void gimbal_ctrl::reset() {
    cmd_ = {};
    flag_.car_ctrl_flag_ = body_state_::E_NOT_FOLLOW;
    flag_.auto_aim_gimbal_ = false;
    flag_.chassis_sate_ = Coordinate::mode_state::E_WAITING;
    flag_.switch_cmd_ = Coordinate::mode_switch_cmd::CMD_WAITING;
    flag_.trigger_mode_ = Gimbal::trigger_mode_e::E_TRIGGER_REST;
    flag_.fric_mode_ = Gimbal::fric_mode_e::E_FRIC_REST;
}

void gimbal_ctrl::mouse_update(mouse_pkg pkg_) {

}

void gimbal_ctrl::rc_update(const bsp_rc_data_t *rc) {
    //cmd更新
    cmd_.vx = rc->rc_l[0]/660.0f;
    cmd_.vy = rc->rc_l[1]/660.0f;
    cmd_.delta_body_yaw = rc->reserved/660.0f;
    cmd_.delta_head_yaw = rc->rc_r[0]/660.0f;
    cmd_.delta_pit = rc->rc_r[1]/660.0f;
    if(rc->s_r == 1 || rc->s_r == 0)
        cmd_.shoot_flag = true;

    //flag更新
    flag_.chassis_sate_ = Coordinate::mode_state::E_WAITING;
    flag_.switch_cmd_ = Coordinate::mode_switch_cmd::CMD_WAITING;
    flag_.car_ctrl_flag_ = body_state_::E_NOT_FOLLOW;

    //flag云台控制逻辑
    if(rc->s_l == 0) {
        flag_.gimbal_ctrl_ = false;
        flag_.auto_aim_gimbal_ = false;
    }
    else if(rc->s_l == -1) {
        flag_.gimbal_ctrl_ = false;
        flag_.auto_aim_gimbal_ = true;
    }
    else if(rc->s_r == 1) {
        flag_.gimbal_ctrl_ = true;
        flag_.auto_aim_gimbal_ = false;
    }
    //flag发射机构控制逻辑
    if(rc->s_r == 0) {
        flag_.shoot_ctrl_ = false;
        flag_.auto_aim_shoot_ = false;
    }
    else if(rc->s_r == 1) {
        flag_.shoot_ctrl_ = true;
        flag_.auto_aim_shoot_ = false;
    }
    else if(rc->s_r == -1) {
        flag_.shoot_ctrl_ = false;
        flag_.auto_aim_shoot_ = true;
    }

    flag_.trigger_mode_ = Gimbal::trigger_mode_e::E_TRIGGER_REST;
    flag_.fric_mode_ = Gimbal::fric_mode_e::E_FRIC_REST;
}

ibc_chassis_send_pkg *gimbal_ctrl::get_chassis_pkg(){
    return &chassis_pkg_;
}

ibc_gimbal_send_pkg *gimbal_ctrl::get_gimbal_pkg(){
    return &gimbal_pkg_;
}

void gimbal_ctrl::tick(){
    //此处先不写哪些乱七八糟的逻辑
    //todo:完成运动控制的映射
    gimbal_pkg_.chassis_cmd_ = flag_.chassis_sate_;
    gimbal_pkg_.delta_yaw_ = cmd_.delta_body_yaw;
    gimbal_pkg_.switch_cmd_ = flag_.switch_cmd_;
    gimbal_pkg_.dot_S = cmd_.vx;
    app_msg_can_send(E_CAN3,GIMBAL_ID,gimbal_pkg_);

}

