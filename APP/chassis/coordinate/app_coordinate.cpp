//
// Created by 15082 on 2026/2/12.
//

#include "app_coordinate.h"

using namespace Coordinate;

void app_coordinate::tick() {
    //更新机器人快照
    robot_snap_ptr_->snap_update();

    //根据当前状态选择不同的模式函数


    //选择对应的模式函数
    (this->*behavior_table_[mode_state_.current_state_])(robot_snap_ptr_, mode_state_);
    //发送扭矩到关节模组，只能在这用
    motor_tor_update();
}

void app_coordinate::motor_tor_update(){
    auto p = &motor_component_;
    auto o = &motor_output_;
    p->j1->set_tor(o->tor_j1);
    p->j2->set_tor(o->tor_j2);
    p->j3->set_tor(o->tor_j3);
    p->j4->set_tor(o->tor_j4);
    p->left->set_tor(o->dynamic_left);
    p->right->set_tor(o->dynamic_right);
}

void app_coordinate::motor_rest(){
    motor_output_ = {};
}

void app_coordinate::exe_any(snap* robot_snap, mode_state_struct state_struct){
    robot_snap->snap_clear();
    motor_rest();
}

void app_coordinate::exe_waiting(snap *robot_snap, mode_state_struct state){
    motor_rest();
}


mode_state_struct app_coordinate::mode_reset(){
    mode_state_struct temp = {
        .extern_cmd_ = CMD_EXECUTING,
        .inner_cmd_ = CMD_EXECUTING,
        .current_state_ = E_WAITING
    };
    return temp;
}

mode_state_struct app_coordinate::mode_ptr_search(){
    mode_state_struct temp = {
    .extern_cmd_ = CMD_EMERGENCY,
    .inner_cmd_ = CMD_EMERGENCY,
    .current_state_ = E_FALL_PROTECT};
    //如果出现了紧急情况直接进入保护
    if(mode_state_.inner_cmd_ == CMD_EMERGENCY || mode_state_.extern_cmd_ == CMD_EMERGENCY)
        return temp;
    //如果没有外部的特殊cmd就继续执行当前目标
    else if(mode_state_.inner_cmd_ == CMD_EXECUTING && mode_state_.extern_cmd_ == CMD_EXECUTING)
        return mode_state_;

    temp = mode_state_;
    for(auto & i : move_map) {
        if(temp.current_state_ == i.current_state &&
            (temp.inner_cmd_ == i.switch_cmd || temp.extern_cmd_ == i.switch_cmd)) {
            temp.current_state_= i.next_state;
            return temp;
        }
    }
    return temp;
}

