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

//根据键盘的pkg更新控制命令的数值和flag
//键盘pkg已经经过了一次处理，直接映射到控制命令和flag上就行了
void gimbal_ctrl::keyboard_update(Gimbal::keyboard_cmd_pkg pkg_) {
    //基础的运动控制系统
    cmd_.vx = pkg_.vx;
    cmd_.vy = pkg_.vy;
    cmd_.delta_body_yaw = pkg_.spin;
    cmd_.delta_head_yaw = pkg_.mouse_x;
    cmd_.delta_pit = pkg_.mouse_y;
    cmd_.target_height = pkg_.target_height;
    //几个flag的更新
    //摩檫轮的更新
    if(pkg_.fric_flag == true)
        flag_.fric_mode_ = Gimbal::fric_mode_e::E_FRIC_FAST;
    else
        flag_.fric_mode_ = Gimbal::fric_mode_e::E_FRIC_REST;

    //拨弹盘的更新,分为三档，0/1/2分别对应不同射速
    if(pkg_.shoot_fre == 0)
        flag_.trigger_mode_ = Gimbal::trigger_mode_e::E_TRIGGER_REST;
    else if(pkg_.shoot_fre == 1)
        flag_.trigger_mode_ = Gimbal::trigger_mode_e::E_TRIGGER_SLOW;
    else if(pkg_.shoot_fre == 2)
        flag_.trigger_mode_ = Gimbal::trigger_mode_e::E_TRIGGER_FAST;

    //自瞄的更新
    flag_.auto_aim_gimbal_ = pkg_.auto_aim_flag;
    flag_.shoot_ctrl_ = pkg_.player_fire;

    //云台reset和云台enable的更新
    flag_.enable = pkg_.reset_flag;
    if(pkg_.gimbal_flag == true) {
        flag_.gimbal_ctrl_ = !flag_.gimbal_ctrl_;
    }


    if(pkg_.switch_rest == true) {
        flag_.switch_cmd_ = Coordinate::mode_switch_cmd::CMD_EMERGENCY;//保护状态，优先级最高
    }
    else if(pkg_.switch_reset == true) {
        flag_.switch_cmd_ = Coordinate::mode_switch_cmd::CMD_REBOOT;//倒地自启
    }
    else if(pkg_.switch_put == true) {
        flag_.switch_cmd_ = Coordinate::mode_switch_cmd::CMD_START;//启动，把腿放下来
    }
    else if(pkg_.switch_lqr == true) {
        flag_.switch_cmd_ = Coordinate::mode_switch_cmd::CMD_NORMAL_LQR;
    }
     else if(pkg_.switch_dog == true) {
        flag_.switch_cmd_ = Coordinate::mode_switch_cmd::CMD_DOG_START;
    }
     else {
        flag_.switch_cmd_ = Coordinate::mode_switch_cmd::CMD_EXECUTING;
    }
}

void gimbal_ctrl::rc_update(const bsp_rc_data_t *rc) {
    //cmd更新
    cmd_.vx = rc->rc_l[0]/660.0f;
    cmd_.vy = rc->rc_l[1]/660.0f;
    cmd_.delta_body_yaw = rc->reserved/660.0f;
    cmd_.delta_head_yaw = rc->rc_r[0]*2/660.0f;
    cmd_.delta_pit = rc->rc_r[1]*2/660.0f;
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
    else if(rc->s_l == 1) {
        flag_.gimbal_ctrl_ = true;
        flag_.auto_aim_gimbal_ = false;
    }
    //flag发射机构控制逻辑
    if(rc->s_r == 0) {
        flag_.shoot_ctrl_ = false;
    }
    else if(rc->s_r == 1) {
        flag_.shoot_ctrl_ = true;
    }
    else if(rc->s_r == -1) {
        flag_.shoot_ctrl_ = false;
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

}

