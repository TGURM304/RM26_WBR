//
// Created by 15082 on 2026/2/12.
//

#include "app_coordinate.h"

#include "bsp_uart.h"
#include "robomaster.h"
#include "robot_data.h"

#include <future>
using namespace Coordinate;

void app_coordinate::tick() {
    //更新机器人快照
    robot_snap_ptr_->snap_update();

    //根据当前状态选择不同的模式函数
    //选择对应的模式函数
    // (this->*behavior_table_[mode_state_.current_state_])(robot_snap_ptr_, mode_state_);
    //发送扭矩到关节模组，只能在这用
    motor_tor_update();
    controller_.dog_ctrl->tick();

    auto snap = robot_snap_ptr_->current_snap_get()->lqr_data;

    //按照频率发送内容,200Hz
    send_cnt++;
    if(send_cnt >= 5) {
        send_cnt = 0;
        auto p = robot_snap_ptr_->current_snap_get();
        chassis_send_.vector_x = IBC::float32_to_uint16(p->robot_raw_data.vector_x,1,-1);
        chassis_send_.vector_y = IBC::float32_to_uint16(p->robot_raw_data.vector_y,1,-1);
        chassis_send_.vector_z = IBC::float32_to_uint16(p->robot_raw_data.vector_z,1,-1);
        chassis_send_.chassis_cmd_ = mode_state_.current_state_;
        app_msg_can_send(E_CAN3,CHASSIS_ID,chassis_send_);
    }

}

void app_coordinate::init(){
    ibc_gimbal_.init();
}

void app_coordinate::test_function(const bsp_rc_data_t *rc){
    auto force = controller_.leg_ctrl->get_output();

    // out_side_cmd_update(ibc_gimbal_);

    tick();

    robot_snap_ptr_->snap_update();
    mode_state_.last_state = mode_state_.current_state_;
    ctrl_struct test = {};
    test.body_height = (rc->rc_l[1]*1.0f)/660.0f;
    test.ver_x =  (rc->rc_r[1]*2.0f)/660.0f;
    test.ver_y = (rc->rc_r[0]*2.0f)/660.0f;

    if(rc->s_l == -1 && rc->s_r == 0) {
        mode_state_.current_state_ = E_DOG;
        exe_dog(robot_snap_ptr_,mode_state_,test);
    }
    else if(rc->s_l == 1 && rc->s_r == 0) {
        mode_state_.current_state_ = E_PUT_LEG;
        exe_put_leg(robot_snap_ptr_,mode_state_,test);
    }
    else if(rc->s_l == 1 && rc->s_r == 1) {
        mode_state_.current_state_ = E_LQR;
        exe_lqr(robot_snap_ptr_,mode_state_,test);
    }
    else {
        mode_state_.current_state_ = E_WAITING;
        motor_rest();
    }
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
    out_side_cmd_update(ibc_gimbal_);
}

//更新外部命令
void app_coordinate::out_side_cmd_update(app_msg_can_receiver<IBC::ibc_gimbal> gimbal){

    // mode_state_.height_record = gimbal()->height;
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

    if((robot_ctrl->leg_ctrl->left_flag == LegController::E_SOFT ||
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

//小板凳控制函数
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

    //此处不关系机体角度phi
    float32_t delta[10];
    delta[0] = -p->lqr_data.S;
    delta[1] = -p->lqr_data.dot_S;
    delta[2] = 0;
    delta[3] = 0;
    delta[4] = 0;
    delta[5] = 0;
    delta[6] = 0;
    delta[7] = 0;
    delta[8] = -p->lqr_data.body_theta;
    delta[9] = -p->lqr_data.body_dot_theta;

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
    motor_output_.dynamic_left = left.wheel_balance + left.wheel_move;
    motor_output_.dynamic_right = right.wheel_balance + right.wheel_move;
}

//正常轮腿的控制部分
void app_coordinate::exe_lqr(snap *robot_snap, mode_state_struct state,ctrl_struct ctrl){

    auto p = robot_snap->current_snap_get();
    // 首次进入lqr模块的时候的进行数据净化
     if(state.last_state != state.current_state_ && state.current_state_ == E_LQR) {
         LQR_target_data.S = 0;
         LQR_target_data.phi = 0;
         LQR_target_data.dot_S = 0;
         LQR_target_data.dot_phi = 0;

         mode_state_.reduce_cnt = 0;
         mode_state_.delta_S = 0;
         mode_state_.height_record = 0.18f;

         robot_snap->snap_clear_S();
         robot_snap->snap_set_zero();
     }

    target_update(robot_snap,ctrl);
    basic_lqr_ctrl(robot_snap,state,ctrl);
    basic_vmc_update(robot_snap,true);

    //更新到目标输出中
    motor_tor_ready();
}

//土狗模式部分
void app_coordinate::exe_dog(snap *robot_snap, mode_state_struct state, ctrl_struct ctrl){
    if(state.last_state != E_DOG && state.current_state_ == E_DOG) {
        controller_.dog_ctrl->clear();
    }
    auto p = robot_snap->current_snap_get();
    auto robot_ctrl = &controller_;


    //step1: 计算目标速度角度和大小
    float target_yaw = gimbal_yaw;
    target_yaw = gimbal_yaw + atan2f(ctrl.ver_y,ctrl.ver_x);
    //我们先默认目标姿态为0，后续可以加上云台
    float delta_yaw = target_yaw - robot_snap->current_snap_get()->robot_raw_data.body_phi;
    if(delta_yaw > PI) delta_yaw -= 2*PI;
    else if(delta_yaw < -PI) delta_yaw += 2*PI;
    float cor_gry = delta_yaw*10.0f;
    if(abs(cor_gry) > 10) {
        cor_gry = cor_gry > 0? 10:-10;
    }
    float ver = sqrtf(ctrl.ver_x*ctrl.ver_x + ctrl.ver_y*ctrl.ver_y);
    if(abs(delta_yaw) < PI/6) {
        ver = ver*cosf(delta_yaw);
    }

    bool ready_flag = false;
    if(p->left_leg.theta > 0.8 && p->left_leg.theta < 1.2
        && p->right_leg.theta > 0.8 && p->right_leg.theta < 1.2) {
        ready_flag = true;
    }
    else {
        if(p->left_leg.theta >1.2 || p->left_leg.theta < 0.8) {
            controller_.leg_ctrl->left_omega_only(p->left_leg,-1);
            controller_.leg_ctrl->left_len_update(p->left_leg,0.17);
        }
        else {
            controller_.leg_ctrl->left_deg_update(p->left_leg,1.06);
            controller_.leg_ctrl->left_len_update(p->left_leg,0.17);
        }
        if(p->right_leg.theta >1.2 || p->right_leg.theta < 0.8) {
            controller_.leg_ctrl->right_omega_only(p->right_leg,-1);
            controller_.leg_ctrl->right_len_update(p->right_leg,0.17);
        }
        else {
            controller_.leg_ctrl->right_deg_update(p->right_leg,1.06);
            controller_.leg_ctrl->right_len_update(p->right_leg,0.17);
        }    }
    if(ready_flag == true) {
        controller_.leg_ctrl->left_deg_update(p->left_leg,1.06);
        controller_.leg_ctrl->left_len_update(p->left_leg,0.17);
        controller_.leg_ctrl->right_deg_update(p->right_leg,1.06);
        controller_.leg_ctrl->right_len_update(p->right_leg,0.17);
    }

    robot_ctrl->left_vmc_pkg.force_y = 0;
    robot_ctrl->left_vmc_pkg.force_x = 0;
    robot_ctrl->right_vmc_pkg.force_y = 0;
    robot_ctrl->right_vmc_pkg.force_x = 0;
    robot_ctrl->left_vmc_pkg.force_L = controller_.leg_ctrl->get_output().force_left;
    robot_ctrl->left_vmc_pkg.leg_tor = controller_.leg_ctrl->get_output().tor_left;
    robot_ctrl->right_vmc_pkg.force_L = controller_.leg_ctrl->get_output().force_right;
    robot_ctrl->right_vmc_pkg.leg_tor = controller_.leg_ctrl->get_output().tor_right;


    robot_ctrl->vmc->tor_clc(robot_ctrl->left_vmc_pkg, p->left_leg, VMC::E_Left);
    robot_ctrl->vmc->tor_clc(robot_ctrl->right_vmc_pkg,p->right_leg, VMC::E_Right);


    if(ready_flag == false) {
        ver = 0;
        cor_gry = 0;
    }
    controller_.dog_ctrl->speed_update(ver,cor_gry,
        p->robot_raw_data.speed_left,p->robot_raw_data.speed_right);

    //step3: 拉取数据并且输出
    auto answer= robot_ctrl->vmc->tor_get();
    motor_output_.tor_j1 = answer.p_right_tor2;
    motor_output_.tor_j2 = answer.p_right_tor1;
    motor_output_.tor_j3 = answer.p_left_tor1 ;
    motor_output_.tor_j4 = answer.p_left_tor2 ;

    auto output = controller_.dog_ctrl->output_get();
    motor_output_.dynamic_left = output.first;
    motor_output_.dynamic_right = output.second;
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

void app_coordinate::basic_lqr_ctrl(snap *robot_snap, mode_state_struct state,ctrl_struct ctrl){
    auto p = robot_snap->current_snap_get();
    auto zero_p = robot_snap->zero_snap_get();

    //这里的phi是机体的基准角度，在这之上叠加一ver_deg用于基于当前状态进行左右平移
    LQR_target_data.phi += ctrl.gry/1000.0f;
    LQR_target_data.phi > PI? LQR_target_data.phi -= 2*PI
        :(LQR_target_data.phi < -PI? LQR_target_data.phi += 2*PI:0);
    //target_ver的计算：根据当前的速度方向和云台的偏转角度来计算出一个目标速度，云台偏转越大，目标速度越小，偏转为0时目标速度最大
    float target_ver = 0;
    float ver_deg = atan2f(ctrl.ver_y,ctrl.ver_x);
    float target_phi = LQR_target_data.phi + ver_deg;
    //把转换后的目标角度转换到[-PI,PI]范围内
    target_phi > PI? target_phi -= 2*PI:(target_phi < -PI? target_phi += 2*PI:0);
    //把差值转换到[-PI,PI]范围内（优弧）
    float delta_phi = target_phi
        - (p->robot_raw_data.body_phi + zero_p->robot_raw_data.body_phi);
    if(delta_phi > PI) delta_phi -= 2*PI;
    else if(delta_phi < -PI) delta_phi += 2*PI;
    //对delta_phi进行限制，防止过大的转向
    if(abs(delta_phi) > PI/6) {
        delta_phi = delta_phi > 0? PI/6:-PI/6;
        target_ver = 0;
    }
    else {
        target_ver = sqrtf(ctrl.ver_x*ctrl.ver_x + ctrl.ver_y*ctrl.ver_y)*cosf(delta_phi);
    }

    LQR_target_data.S += target_ver/1000.0f;
    LQR_target_data.dot_S = target_ver;

    LQR_target_data.phi > PI? LQR_target_data.phi -= 2*PI:(LQR_target_data.phi < -PI? LQR_target_data.phi += 2*PI:0);
    LQR_target_data.dot_phi = ctrl.gry;

    //腿长位移补偿
    auto x =  mode_state_.height_record;
    auto dis = 5.6651f - 40.5217f*x + 124.487f*powf(x,2) - 128.703f*powf(x,3);

    float32_t delta[10];
    delta[0] =  LQR_target_data.S       - (p->lqr_data.S                    -   zero_p->lqr_data.S - dis);
    delta[1] =  LQR_target_data.dot_S    - (p->lqr_data.dot_S-   zero_p->lqr_data.dot_S);
    delta[2] =  delta_phi;
    // float temp = (p->lqr_data.phi              -zero_p->lqr_data.phi            );
    // temp > PI? temp -= 2*PI:(temp < -PI? temp += 2*PI:0);
    // delta[2] =  LQR_target_data.phi     - temp;
    // delta[2] > PI? delta[2] -= 2*PI:(delta[2] < -PI? delta[2] += 2*PI:0);
    // delta[2] = (((delta[2]) > (1)) ? (1) : (((delta[2]) < -(1)) ? -(1) : (delta[2])));
    delta[3] =  LQR_target_data.dot_phi  - (p->lqr_data.dot_phi          -zero_p->lqr_data.dot_phi        );
    delta[4] =  0                       - (p->lqr_data.left_theta      );
    delta[5] =  0                       - (p->lqr_data.left_dot_theta  );
    delta[6] =  0                       - (p->lqr_data.right_theta     );
    delta[7] =  0                       - (p->lqr_data.right_dot_theta );
    delta[8] =  0                       - (p->lqr_data.body_theta      );
    delta[9] =  0                       - (p->lqr_data.body_dot_theta  );

    controller_.lqr_controller->fit_clc(delta,p->left_leg.L0,p->right_leg.L0);
}

//一种基本的VMC提交
void app_coordinate::basic_vmc_update(snap *robot_snap,bool forward){
    auto p = robot_snap->current_snap_get();
    auto robot_ctrl = &controller_;
    auto left =  controller_.lqr_controller->get_lqr_output(LQR::E_left);
    auto right = controller_.lqr_controller->get_lqr_output(LQR::E_right);

    if(forward == true) {
        robot_ctrl->left_vmc_pkg.force_y = LEG_FORWARD(mode_state_.height_record);
        robot_ctrl->left_vmc_pkg.force_x  = 0;
        robot_ctrl->right_vmc_pkg.force_y = LEG_FORWARD(mode_state_.height_record);
        robot_ctrl->right_vmc_pkg.force_x = 0;
    }
    else {
        robot_ctrl->left_vmc_pkg.force_y = 0;
        robot_ctrl->left_vmc_pkg.force_x = 0;
        robot_ctrl->right_vmc_pkg.force_y = 0;
        robot_ctrl->right_vmc_pkg.force_x = 0;
    }

    robot_ctrl->left_vmc_pkg.force_L = controller_.leg_ctrl->get_output().force_left;
    robot_ctrl->left_vmc_pkg.leg_tor = left.body_balance + left.body_move;
    robot_ctrl->right_vmc_pkg.force_L = controller_.leg_ctrl->get_output().force_right;
    robot_ctrl->right_vmc_pkg.leg_tor = right.body_balance+right.body_move;

    robot_ctrl->vmc->tor_clc(robot_ctrl->left_vmc_pkg, p->left_leg, VMC::E_Left);
    robot_ctrl->vmc->tor_clc(robot_ctrl->right_vmc_pkg,p->right_leg, VMC::E_Right);
}

//更新所有电机扭矩到准备区
void app_coordinate::motor_tor_ready(){
    auto robot_ctrl = &controller_;
    auto left =  controller_.lqr_controller->get_lqr_output(LQR::E_left);
    auto right = controller_.lqr_controller->get_lqr_output(LQR::E_right);

    auto answer= robot_ctrl->vmc->tor_get();
    motor_output_.tor_j1 = answer.p_right_tor2 + answer.c_right_tor2;
    motor_output_.tor_j2 = answer.p_right_tor1 + answer.c_right_tor1;
    motor_output_.tor_j3 = answer.p_left_tor1 + answer.c_left_tor1;
    motor_output_.tor_j4 = answer.p_left_tor2 + answer.c_left_tor2;

    motor_output_.dynamic_left = left.wheel_balance + left.wheel_move;
    motor_output_.dynamic_right = right.wheel_balance + right.wheel_move;
}

//更新腿长相关内容
void app_coordinate::target_update(snap *robot_snap,ctrl_struct ctrl){
    auto p = robot_snap->current_snap_get();

    mode_state_.height_record += ctrl.body_height/1000.0f;
    if(mode_state_.height_record > HEIGHT_MAX) mode_state_.height_record = HEIGHT_MAX;
    else if(mode_state_.height_record < HEIGHT_MIN) mode_state_.height_record = HEIGHT_MIN;
    controller_.leg_ctrl->left_len_update(p->left_leg,mode_state_.height_record);
    controller_.leg_ctrl->right_len_update(p->right_leg,mode_state_.height_record);
}

