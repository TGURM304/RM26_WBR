//
// Created by 15082 on 2026/2/12.
//

#include "app_coordinate.h"

#include "bsp_uart.h"
#include "robot_data.h"
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
    ctrl_struct test = {};
    if(rc->s_l == 0 && rc->s_r == -1) {
        mode_state_.current_state_ = E_PUT_BODY;
        exe_put_body(robot_snap_ptr_,mode_state_,test);
    }
    else if(rc->s_l == 1 && rc->s_r == -1) {
        mode_state_.current_state_ = E_PUT_LEG;
        exe_put_leg(robot_snap_ptr_,mode_state_,test);
    }
    else {
        mode_state_.current_state_ = E_WAITING;
        motor_rest();
    }

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

void app_coordinate::exe_any(snap* robot_snap, mode_state_struct state_struct,ctrl_struct ctrl){
    robot_snap->snap_clear();
    motor_rest();
}

void app_coordinate::exe_waiting(snap *robot_snap, mode_state_struct state,ctrl_struct ctrl) {
    motor_rest();
}

void app_coordinate::exe_put_body(snap *robot_snap, mode_state_struct state,ctrl_struct ctrl) {
    auto p = robot_snap->current_snap_get();
    auto robot_ctrl = &controller_;

    robot_ctrl->left_vmc_pkg.force_x = 0;
    robot_ctrl->left_vmc_pkg.force_y = 0;
    robot_ctrl->right_vmc_pkg.force_x = 0;
    robot_ctrl->right_vmc_pkg.force_y = 0;

    //由于万向节死锁等一系列奇怪原因，这里只能用一些特殊方法来一起判断
    if(p->robot_raw_data.vector_z > 0.7) {
        controller_.leg_ctrl->leg_clear();

        robot_ctrl->left_vmc_pkg.leg_tor = 0;
        robot_ctrl->left_vmc_pkg.force_L = 0;

        robot_ctrl->right_vmc_pkg.leg_tor = 0;
        robot_ctrl->right_vmc_pkg.force_L = 0;
    }
    else {
        /*
         * 翻身判定：
         * 状态1：腿部归位
         * 状态2：同步翻身
         * 状态1的要求：腿的角度差大或者两个都没有堵转，第一次切换
         * 切换至状态2的要求：腿角度差小于某种值并且腿两个都堵转，或者一只腿堵转同时自由腿超前于堵转腿一个角度
         */
        uint8_t body_flag = 0;//1代表硬模式，2代表软模式，0代表不动
        float dir = 0;
        //判断方向
        if(p->robot_raw_data.vector_z <= 0.7 &&
            p->robot_raw_data.body_theta > 0 && p->robot_raw_data.body_theta < PI*5/6) {
            dir = 1;
        }
        else{ dir = -1;}
        //翻身控制标志位判断
        float delta = p->left_leg.theta- p->right_leg.theta;
        delta < -PI? delta+= 2*PI:(delta>PI?delta-=2*PI:delta);
        if((abs(delta) < 0.2 && abs(robot_ctrl->left_vmc_pkg.leg_tor) > 1.5 && abs(robot_ctrl->right_vmc_pkg.leg_tor) > 1.5)
            || (abs(robot_ctrl->left_vmc_pkg.leg_tor) > 1.5 || abs(robot_ctrl->right_vmc_pkg.leg_tor) > 1.5)) {
            if(abs(robot_ctrl->left_vmc_pkg.leg_tor) > 1.5 && -delta*dir >0.2) {
                body_flag = 1;
            }
            else if(abs(robot_ctrl->right_vmc_pkg.leg_tor) > 1.5 && delta*dir >0.2) {
                body_flag = 1;
            }
            else if((abs(delta) < 0.2 && abs(robot_ctrl->left_vmc_pkg.leg_tor) > 1.5 && abs(robot_ctrl->right_vmc_pkg.leg_tor) > 1.5)) {
                body_flag = 1;
            }
        }
        else {
            body_flag = 2;
        }
        if(body_flag == 1 &&
            (robot_ctrl->leg_ctrl->left_flag == LegController::E_SOFT ||
            robot_ctrl->leg_ctrl->right_flag == LegController::E_SOFT)) {
            robot_ctrl->leg_ctrl->left_flag = LegController::E_HARD;
            robot_ctrl->leg_ctrl->right_flag = LegController::E_HARD;
            robot_ctrl->leg_ctrl->left_omega_write_param(3,2.0/1000.0f,0,15,12);
            robot_ctrl->leg_ctrl->right_omega_write_param(3,2.0/1000.0f,0,15,12);
        }
        else if(body_flag == 2 &&
            (robot_ctrl->leg_ctrl->left_flag == LegController::E_HARD ||
            robot_ctrl->leg_ctrl->right_flag == LegController::E_HARD)) {
            robot_ctrl->leg_ctrl->left_flag = LegController::E_SOFT;
            robot_ctrl->leg_ctrl->right_flag = LegController::E_SOFT;
            robot_ctrl->leg_ctrl->left_omega_write_param(3,2.0/1000.0f,0,2,2);
            robot_ctrl->leg_ctrl->right_omega_write_param(3,2.0/1000.0f,0,2,2);
        }

        robot_ctrl->leg_ctrl->left_omega_only(p->left_leg,dir-delta);
        robot_ctrl->leg_ctrl->right_omega_only(p->right_leg,dir+delta);
        robot_ctrl->leg_ctrl->left_len_update(p->left_leg,0.35);
        robot_ctrl->leg_ctrl->right_len_update(p->right_leg,0.35);

        auto out = robot_ctrl->leg_ctrl->get_output();
        robot_ctrl->left_vmc_pkg.leg_tor = out.tor_left;
        robot_ctrl->left_vmc_pkg.force_L = out.force_left;

        robot_ctrl->right_vmc_pkg.leg_tor = out.tor_right;
        robot_ctrl->right_vmc_pkg.force_L = out.force_right;
    }

    //计算PKG中目标对应的电机扭矩
    robot_ctrl->vmc->tor_clc(robot_ctrl->left_vmc_pkg,p->left_leg,VMC::E_Left);
    robot_ctrl->vmc->tor_clc(robot_ctrl->right_vmc_pkg,p->right_leg,VMC::E_Right);

    bsp_uart_printf(E_UART_DEBUG,"%f,%f\r\n",
        robot_ctrl->left_vmc_pkg.leg_tor
        ,robot_ctrl->right_vmc_pkg.leg_tor);

    //更新到目标输出中
    auto answer = robot_ctrl->vmc->tor_get();
    motor_output_.tor_j1 = answer.p_right_tor2;
    motor_output_.tor_j2 = answer.p_right_tor1;
    motor_output_.tor_j3 = answer.p_left_tor1;
    motor_output_.tor_j4 = answer.p_left_tor2;
    motor_output_.dynamic_left  = 0;
    motor_output_.dynamic_right = 0;
}

void app_coordinate::exe_put_leg(snap *robot_snap, mode_state_struct state,ctrl_struct ctrl){
    auto robot_ctrl = &controller_;
    auto p = robot_snap->current_snap_get();

    if(     (robot_ctrl->leg_ctrl->left_flag == LegController::E_SOFT ||
            robot_ctrl->leg_ctrl->right_flag == LegController::E_SOFT)) {
        robot_ctrl->leg_ctrl->left_flag = LegController::E_HARD;
        robot_ctrl->leg_ctrl->right_flag = LegController::E_HARD;
        robot_ctrl->leg_ctrl->left_omega_write_param(3,2.0/1000.0f,0,15,10);
        robot_ctrl->leg_ctrl->right_omega_write_param(3,2.0/1000.0f,0,12,10);
    }
    if(p->left_leg.theta < 0.3 || p->left_leg.theta > 3) {
        controller_.leg_ctrl->left_omega_only(p->left_leg,-1);
        controller_.leg_ctrl->left_leg_len_clear();
        robot_ctrl->left_vmc_pkg.force_L = 0;
    }
    else if(p->left_leg.theta > 0.1 && p->left_leg.theta< 3.1) {
        controller_.leg_ctrl->left_deg_update(p->left_leg,PI/2);
        controller_.leg_ctrl->left_len_update(p->left_leg,0.17);
        robot_ctrl->left_vmc_pkg.force_L = controller_.leg_ctrl->get_output().force_left;
    }
    if(p->right_leg.theta<0.3 || p->right_leg.theta > 3) {
        controller_.leg_ctrl->right_omega_only(p->right_leg,-1);
        controller_.leg_ctrl->right_leg_len_clear();
        robot_ctrl->right_vmc_pkg.force_L = 0;
    }
    else if(p->right_leg.theta > 0.1 && p->right_leg.theta < 3.1) {
        controller_.leg_ctrl->right_deg_update(p->right_leg,PI/2);
        controller_.leg_ctrl->right_len_update(p->right_leg,0.17);
        robot_ctrl->right_vmc_pkg.force_L = controller_.leg_ctrl->get_output().force_right;
    }

    //更新PID数据到PKG中
    robot_ctrl->left_vmc_pkg.force_x = 0;
    robot_ctrl->left_vmc_pkg.force_y = 0;
    robot_ctrl->left_vmc_pkg.leg_tor = controller_.leg_ctrl->get_output().tor_left;
    robot_ctrl->right_vmc_pkg.force_x = 0;
    robot_ctrl->right_vmc_pkg.force_y = 0;
    robot_ctrl->right_vmc_pkg.leg_tor = controller_.leg_ctrl->get_output().tor_right;

    //计算PKG中目标对应的电机扭矩
    robot_ctrl->vmc->tor_clc(robot_ctrl->left_vmc_pkg,p->left_leg,VMC::E_Left);
    robot_ctrl->vmc->tor_clc(robot_ctrl->right_vmc_pkg,p->right_leg,VMC::E_Right);

    //更新到目标输出中
    auto answer = robot_ctrl->vmc->tor_get();
    motor_output_.tor_j1 = answer.p_right_tor2;
    motor_output_.tor_j2 = answer.p_right_tor1;
    motor_output_.tor_j3 = answer.p_left_tor1;
    motor_output_.tor_j4 = answer.p_left_tor2;
    motor_output_.dynamic_left  = 0;
    motor_output_.dynamic_right = 0;
}

void app_coordinate::exe_chair(snap *robot_snap, mode_state_struct state,ctrl_struct ctrl){
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
    controller_.lqr_controller->dynamic_clc(delta,p->left_leg,p->right_leg);

    auto robot_ctrl = &controller_;

    robot_ctrl->left_vmc_pkg.force_y = 0;
    robot_ctrl->left_vmc_pkg.force_x = 0;
    robot_ctrl->right_vmc_pkg.force_y = 0;
    robot_ctrl->right_vmc_pkg.force_x = 0;

    robot_ctrl->left_vmc_pkg.force_L = controller_.leg_ctrl->get_output().force_left;
    robot_ctrl->left_vmc_pkg.leg_tor = controller_.leg_ctrl->get_output().tor_left;
    robot_ctrl->right_vmc_pkg.force_L = controller_.leg_ctrl->get_output().force_right;
    robot_ctrl->right_vmc_pkg.leg_tor = controller_.leg_ctrl->get_output().tor_right;

    robot_ctrl->vmc->tor_clc(robot_ctrl->left_vmc_pkg,p->left_leg,VMC::E_Left);
    robot_ctrl->vmc->tor_clc(robot_ctrl->right_vmc_pkg,p->right_leg,VMC::E_Right);

    //更新到目标输出中
    auto answer = robot_ctrl->vmc->tor_get();
    motor_output_.tor_j1 = answer.p_right_tor2;
    motor_output_.tor_j2 = answer.p_right_tor1;
    motor_output_.tor_j3 = answer.p_left_tor1;
    motor_output_.tor_j4 = answer.p_left_tor2;
    auto left =  controller_.lqr_controller->get_lqr_output(LQR::E_left);
    auto right = controller_.lqr_controller->get_lqr_output(LQR::E_right);
    motor_output_.dynamic_left = left.wheel_balance+left.wheel_move;
    motor_output_.dynamic_right = right.wheel_balance + right.wheel_move;
}

void app_coordinate::exe_lqr(snap *robot_snap, mode_state_struct state,ctrl_struct ctrl){
    auto snap = robot_snap->current_snap_get();

    //腿部控制代码块

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
            temp.current_state_ = i.next_state;
            return temp;
        }
    }
    return temp;
}

/*
 * roll_rad机体roll角，和lqr建模中定义不同，是从后往前看的时候，逆时针为正的角度坐标系
 * 返回值中，first是左腿目标长度
 * second是右腿目标长度
 */
std::pair<float32_t, float32_t> app_coordinate::roll_feed(float32_t roll_rad, float32_t left_r, float32_t right_r, float32_t target_height){
    std::pair<float32_t, float32_t> target;
    target.first = 0, target.second = 0;
    float32_t gama_tan = (left_r-right_r)/RL;
    float32_t beta_deg = atanf(gama_tan)+roll_rad;
    target.first = (2*target_height - RL*tanf(beta_deg))/2;
    target.second = (2*target_height + RL*tanf(beta_deg))/2;
    return  target;
}
