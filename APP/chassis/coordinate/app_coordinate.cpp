//
// Created by 15082 on 2026/2/12.
//

#include "app_coordinate.h"

#include "bsp_uart.h"

using namespace Coordinate;

void app_coordinate::tick() {
    //更新机器人快照
    robot_snap_ptr_->snap_update();

    //根据当前状态选择不同的模式函数
    motor_rest();
    //选择对应的模式函数
    // (this->*behavior_table_[mode_state_.current_state_])(robot_snap_ptr_, mode_state_);
    //发送扭矩到关节模组，只能在这用
    motor_tor_update();
}

void app_coordinate::test_function(const bsp_rc_data_t *rc){
    robot_snap_ptr_->snap_update();
    mode_state_.last_state = mode_state_.current_state_;
    if(rc->s_l == 1 && rc->s_r == 0) {
        mode_state_.current_state_ = E_PUT_BODY;
        exe_put_body(robot_snap_ptr_,mode_state_);
    }
    else if(rc->s_l == 1 && rc->s_r == 1) {
        mode_state_.current_state_ = E_CHAIR;
        exe_chair(robot_snap_ptr_,mode_state_);
    }
    else {
        mode_state_.current_state_ = E_WAITING;
        motor_rest();
    }

   auto p = robot_snap_ptr_->current_snap_get();
    bsp_uart_printf(E_UART_DEBUG,"%f,%f,%f,%f\r\n",
        p->lqr_data.S,p->lqr_data.dot_S,
        p->lqr_data.phi,p->lqr_data.dot_phi);
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

void app_coordinate::exe_waiting(snap *robot_snap, mode_state_struct state) {
    motor_rest();
}

void app_coordinate::exe_put_body(snap *robot_snap, mode_state_struct state){

    auto p = robot_snap->current_snap_get();
    auto ctrl = &controller_;
    if(p->left_leg.theta < 0.3 || p->left_leg.theta > 3) {
        controller_.leg_ctrl->left_omega_only(p->left_leg,-1);
        controller_.leg_ctrl->left_leg_len_clear();
        ctrl->left_vmc_pkg.force_L = 0;
    }
    else if(p->left_leg.theta > 0.1 && p->left_leg.theta< 3.1) {
        controller_.leg_ctrl->left_deg_update(p->left_leg,PI/2);
        controller_.leg_ctrl->left_len_update(p->left_leg,0.17);
        ctrl->left_vmc_pkg.force_L = controller_.leg_ctrl->get_output().force_left;
    }
    if(p->right_leg.theta<0.3 || p->right_leg.theta > 3) {
        controller_.leg_ctrl->right_omega_only(p->right_leg,-1);
        controller_.leg_ctrl->right_leg_len_clear();
        ctrl->right_vmc_pkg.force_L = 0;
    }
    else if(p->right_leg.theta > 0.1 && p->right_leg.theta < 3.1) {
        controller_.leg_ctrl->right_deg_update(p->right_leg,PI/2);
        controller_.leg_ctrl->right_len_update(p->right_leg,0.17);
        ctrl->right_vmc_pkg.force_L = controller_.leg_ctrl->get_output().force_right;
    }

    //更新PID数据到PKG中
    ctrl->left_vmc_pkg.force_x = 0;
    ctrl->left_vmc_pkg.force_y = 0;
    ctrl->left_vmc_pkg.leg_tor = controller_.leg_ctrl->get_output().tor_left;
    ctrl->right_vmc_pkg.force_x = 0;
    ctrl->right_vmc_pkg.force_y = 0;
    ctrl->right_vmc_pkg.leg_tor = controller_.leg_ctrl->get_output().tor_right;

    //计算PKG中目标对应的电机扭矩
    ctrl->vmc->tor_clc(ctrl->left_vmc_pkg,p->left_leg,VMC::E_Left);
    ctrl->vmc->tor_clc(ctrl->right_vmc_pkg,p->right_leg,VMC::E_Right);

    //更新到目标输出中
    auto answer = ctrl->vmc->tor_get();
    motor_output_.tor_j1 = answer.p_right_tor2;
    motor_output_.tor_j2 = answer.p_right_tor1;
    motor_output_.tor_j3 = answer.p_left_tor1;
    motor_output_.tor_j4 = answer.p_left_tor2;
    motor_output_.dynamic_left  = 0;
    motor_output_.dynamic_right = 0;
}

void app_coordinate::exe_chair(snap *robot_snap, mode_state_struct state){
    if(state.current_state_ == E_CHAIR && state.last_state != E_CHAIR) {
        robot_snap->snap_clear_S();
        robot_snap->snap_set_zero();
    }

    auto p = robot_snap->current_snap_get();
    controller_.leg_ctrl->left_deg_update(p->left_leg,PI/2);
    controller_.leg_ctrl->left_len_update(p->left_leg,0.17);
    controller_.leg_ctrl->right_deg_update(p->right_leg,PI/2);
    controller_.leg_ctrl->right_len_update(p->right_leg,0.17);

    float32_t delta[10];
    delta[0] = -p->lqr_data.S;
    delta[1] = -p->lqr_data.dot_S;
    // delta[2] = -p->lqr_data.phi;
    // delta[3] = -p->lqr_data.dot_phi;
    delta[4] = 0;
    delta[5] = 0;
    delta[6] = 0;
    delta[7] = 0;
    delta[8] = -p->lqr_data.body_theta;
    delta[9] = -p->lqr_data.body_dot_theta;

    // delta[0] = 0;
    // delta[1] = 0;
    delta[2] = 0;
    delta[3] = 0;
    // delta[4] = 0;
    // delta[5] = 0;
    // delta[6] = 0;
    // delta[7] = 0;
    // delta[8] = 0;
    // delta[9] = 0;

    controller_.lqr_controller->static_clc(delta);

    auto ctrl = &controller_;

    // ctrl->left_vmc_pkg.force_y = 40*cos(p->lqr_data.body_theta);
    // ctrl->left_vmc_pkg.force_x = -40*sin(p->lqr_data.body_theta);
    // ctrl->right_vmc_pkg.force_y = 40*cos(p->lqr_data.body_theta);
    // ctrl->right_vmc_pkg.force_x = -40*sin(p->lqr_data.body_theta);

    ctrl->left_vmc_pkg.force_y = 0;
    ctrl->left_vmc_pkg.force_x = 0;
    ctrl->right_vmc_pkg.force_y = 0;
    ctrl->right_vmc_pkg.force_x = 0;

    ctrl->left_vmc_pkg.force_L = controller_.leg_ctrl->get_output().force_left;
    ctrl->left_vmc_pkg.leg_tor = controller_.leg_ctrl->get_output().tor_left;
    ctrl->right_vmc_pkg.force_L = controller_.leg_ctrl->get_output().force_right;
    ctrl->right_vmc_pkg.leg_tor = controller_.leg_ctrl->get_output().tor_right;

    ctrl->vmc->tor_clc(ctrl->left_vmc_pkg,p->left_leg,VMC::E_Left);
    ctrl->vmc->tor_clc(ctrl->right_vmc_pkg,p->right_leg,VMC::E_Right);

    //更新到目标输出中
    auto answer = ctrl->vmc->tor_get();
    motor_output_.tor_j1 = answer.p_right_tor2;
    motor_output_.tor_j2 = answer.p_right_tor1;
    motor_output_.tor_j3 = answer.p_left_tor1;
    motor_output_.tor_j4 = answer.p_left_tor2;
    auto left =  controller_.lqr_controller->get_lqr_output(LQR::E_left);
    auto right = controller_.lqr_controller->get_lqr_output(LQR::E_right);
    // motor_output_.dynamic_left = 0;
    // motor_output_.dynamic_right = 0;
    motor_output_.dynamic_left = left.wheel_balance+left.wheel_move;
    motor_output_.dynamic_right = right.wheel_balance + right.wheel_move;
}

void app_coordinate::exe_lqr(snap *robot_snap, mode_state_struct state){

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

