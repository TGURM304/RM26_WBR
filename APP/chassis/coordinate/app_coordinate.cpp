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
    //选择对应的模式函数
    // (this->*behavior_table_[mode_state_.current_state_])(robot_snap_ptr_, mode_state_);
    //发送扭矩到关节模组，只能在这用

    motor_tor_update();

    //离地检测测试代码
    //腿推力上面还有腿xy轴方向上的推力
    float left_force = controller_.left_vmc_pkg.force_L;
    float left_tor = controller_.left_vmc_pkg.leg_tor;
    float right_force = controller_.right_vmc_pkg.force_L;
    float right_tor = controller_.right_vmc_pkg.leg_tor;

    support_.leg_data_update(robot_snap_ptr_->current_snap_get(),
        robot_snap_ptr_->observer_get(),
        left_force,left_tor,right_force,right_tor);
    support_.support_clc(Coordinate::E_left);
    support_.support_clc(Coordinate::E_right);
    float32_t left_support = support_.get_support()->left_support_ + LEG_FORWARD(mode_state_.height_record);
    float32_t right_support = support_.get_support()->right_support_ + LEG_FORWARD(mode_state_.height_record);
    if((left_support < support_.get_support()->support_limit_up_ && robot_snap_ptr_->current_snap_get()->left_leg.L0 > 0.22) &&
        right_support < support_.get_support()->support_limit_up_ && robot_snap_ptr_->current_snap_get()->right_leg.L0 > 0.22) {
        if(cnt_ < 100)
            cnt_ ++;
        else if(cnt_ == 100)
            off_ground_flag_ = true;
    }
    else if(left_support > support_.get_support()->support_limit_down_ ||
        right_support > support_.get_support()->support_limit_down_) {
        if(cnt_ > 0)
            cnt_--;
        else if(cnt_ == 0)
            off_ground_flag_ = false;
    }

    //按照频率发送内容
    send_cnt++;
    if(send_cnt >= 5) {
        send_cnt = 0;
        auto p = robot_snap_ptr_->current_snap_get();
        chassis_send_.vector_x = p->robot_raw_data.vector_x;
        chassis_send_.vector_y = p->robot_raw_data.vector_y;
        chassis_send_.vector_z = p->robot_raw_data.vector_z;
        chassis_send_.mode = mode_state_.current_state_;
        app_msg_can_send(E_CAN3,CHASSIS_ID,chassis_send_);
    }

}

void app_coordinate::init(){
    ibc_gimbal_.init();
}

void app_coordinate::test_function(const bsp_rc_data_t *rc){
    auto temp = support_.get_support();
    auto force = controller_.leg_ctrl->get_output();

    app_msg_vofa_send(E_UART_DEBUG,
        temp->left_support_,
        temp->right_support_,
        force.force_left,
        force.force_right,
        mode_state_.height_record,
        robot_snap_ptr_->current_snap_get()->left_leg.L0,
        cnt_);

    tick();

    // mode_state_.last_state = mode_state_.current_state_;
    // ctrl_struct test = {};
    // test.ver_x = (rc->rc_l[1]*1.0f)/660.0f;
    // test.ver_y = -(rc->rc_l[0]*1.0f)/660.0f;
    //
    // if(rc->s_l == 1) {
    //     mode_state_.current_state_ = E_DOG;
    //     exe_dog(robot_snap_ptr_,mode_state_,test);
    // }
    // else {
    //     mode_state_.current_state_ = E_WAITING;
    //     motor_rest();
    // }

    robot_snap_ptr_->snap_update();
    mode_state_.last_state = mode_state_.current_state_;
    ctrl_struct test = {};
    test.speed = (rc->rc_r[1]*1.0f)/660.0f;
    test.gry = -(rc->reserved*1.0f)/660.0f;
    test.body_height = (rc->rc_l[1]*1.0f)/660.0f/5.0f;

    if(rc->s_l == 0 && rc->s_r == -1) {
        mode_state_.current_state_ = E_PUT_BODY;
        exe_put_body(robot_snap_ptr_,mode_state_,test);
    }
    else if(rc->s_l == 1 && rc->s_r == -1) {
        mode_state_.current_state_ = E_PUT_LEG;
        exe_put_leg(robot_snap_ptr_,mode_state_,test);
    }
    else if(rc->s_l == 1 && rc->s_r == 0) {
        mode_state_.current_state_ = E_CHAIR;
        exe_chair(robot_snap_ptr_,mode_state_,test);
    }
    else if(rc->s_l == 1 && rc->s_r == 1) {
        mode_state_.current_state_ = E_LQR;
        exe_lqr(robot_snap_ptr_,mode_state_,test);
    }
    else {
        mode_state_.current_state_ = E_WAITING;
        motor_rest();
    }

    // auto snap = robot_snap_ptr_->current_snap_get()->lqr_data;
    // auto ls = controller_.lqr_controller->get_lqr_output(LQR::E_left);
    // auto rs = controller_.lqr_controller->get_lqr_output(LQR::E_right);
    // auto ld = controller_.lqr_controller->get_dynamic(LQR::E_left);
    // auto rd = controller_.lqr_controller->get_dynamic(LQR::E_right);
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
    auto zero_p = robot_snap->zero_snap_get();

    // 首次进入lqr模块的时候的进行数据净化
     if(state.last_state != state.current_state_ && state.current_state_ == E_LQR) {
         LQR_target_data.S = 1;
         LQR_target_data.phi = 0;
         LQR_target_data.dot_S = 0;
         LQR_target_data.dot_phi = 0;

         mode_state_.reduce_cnt = 0;
         mode_state_.delta_S = 0;
         mode_state_.height_record = 0.20f;

         robot_snap->snap_clear_S();
         robot_snap->snap_set_zero();
     }

    mode_state_.height_record += ctrl.body_height/1000.0f;
     if(mode_state_.height_record > HEIGHT_MAX) mode_state_.height_record = HEIGHT_MAX;
     else if(mode_state_.height_record < HEIGHT_MIN) mode_state_.height_record = HEIGHT_MIN;
    //roll轴补偿,此处的roll轴应该是和轮腿坐标方向是相反的，first是左腿目标长度，second是右腿目标长度
    // auto leg_target =  roll_feed(-snap->robot_raw_data.body_roll,snap->left_leg.L0,snap->right_leg.L0,ctrl.body_height);
    // controller_.leg_ctrl->left_len_update(snap->left_leg,leg_target.first/sinf(snap->left_leg.theta));
    // controller_.leg_ctrl->right_len_update(snap->right_leg,leg_target.second/sinf(snap->right_leg.theta));
    controller_.leg_ctrl->left_len_update(p->left_leg,mode_state_.height_record);
    controller_.leg_ctrl->right_len_update(p->right_leg,mode_state_.height_record);

    /*我们需要在基础控制的前提下引入柔顺速度控制和位置控制转换
     * 一下几种情况我们需要进行柔顺过度（引入衰减器）
     * 1. 当我们的delta_S误差过大
     * 可能是因为，打滑了、顶到墙了
     * 打滑了我们就得捕捉到一两次误差爆掉就得引入衰减器，并且判断一段之后退出，而顶墙可以使用速度误差判断退出
     * 2. delta_dot_S误差过大
     * 对于顶墙，我们的误差值基本确定，所以说我们不考虑顶墙，这个主要是打滑造成的，
     * 我们要做的是解决打滑问题，检测出来并且实现衰减和退出判断
     *
     * 解决办法：
     * 当检测到delta_S delta_dot_S中有一个爆了，那就直接进入衰减模式，退出衰减的判断条件就是delta_dot_S小到某个值
     */
    //以下为衰减保护相关代码
    //衰减系数可以调节，公式为：k=h^(1/n)，n为衰减次数，k为每次衰减系数，h为最终衰减到的值，也就是说经过n次迭代后衰减到h的值
    //此处我们取h = 0.3， n = 300， k= 0.996
    // if(mode_state_.reduce_cnt >= REDUCE_EDGE_DOWN) {
    //     mode_state_.delta_S *= 0.996f;
    //     LQR_target_data.S = mode_state_.delta_S + (p->lqr_data.S -zero_p->lqr_data.S);
    // }
    // else {
    //     mode_state_.delta_S = LQR_target_data.S - (p->lqr_data.S -zero_p->lqr_data.S);
    //     LQR_target_data.S += ctrl.speed/1000.0f;
    // }
    // if( abs(ctrl.speed) >= 3.0f ||
    //     abs(mode_state_.delta_S) > DELTA_S_EDGE ||
    //     abs(LQR_target_data.dot_S - (p->lqr_data.dot_S-zero_p->lqr_data.dot_S)) > DELTA_VER_EDGE
    //     ) {
    //     if(mode_state_.reduce_cnt < REDUCE_EDGE_UP) {
    //     mode_state_.reduce_cnt += 10;
    //     }
    // }
    // else if(abs(LQR_target_data.dot_S - (p->lqr_data.dot_S - zero_p->lqr_data.dot_S)) < 0.5f
    //      && mode_state_.reduce_cnt > 0)
    //    mode_state_.reduce_cnt -= 1;

    LQR_target_data.S += ctrl.speed/1000.0f;

    LQR_target_data.dot_S = ctrl.speed;
    LQR_target_data.phi += ctrl.gry/1000.0f;

    LQR_target_data.phi > PI? LQR_target_data.phi -= 2*PI:(LQR_target_data.phi < -PI? LQR_target_data.phi += 2*PI:0);
    LQR_target_data.dot_phi = ctrl.gry;

    float32_t delta[10];
    delta[0] =  LQR_target_data.S       - (p->lqr_data.S                -zero_p->lqr_data.S              );
    delta[1] =  LQR_target_data.dot_S    - (p->lqr_data.dot_S            -zero_p->lqr_data.dot_S          );
    float temp = (p->lqr_data.phi              -zero_p->lqr_data.phi            );
    temp > PI? temp -= 2*PI:(temp < -PI? temp += 2*PI:0);
    delta[2] =  LQR_target_data.phi     - temp;
    delta[2] > PI? delta[2] -= 2*PI:(delta[2] < -PI? delta[2] += 2*PI:0);
    delta[2] = (((delta[2]) > (1)) ? (1) : (((delta[2]) < -(1)) ? -(1) : (delta[2])));
    delta[3] =  LQR_target_data.dot_phi  - (p->lqr_data.dot_phi          -zero_p->lqr_data.dot_phi        );
    delta[4] =  0                       - (p->lqr_data.left_theta      );
    delta[5] =  0                       - (p->lqr_data.left_dot_theta  );
    delta[6] =  0                       - (p->lqr_data.right_theta     );
    delta[7] =  0                       - (p->lqr_data.right_dot_theta );
    delta[8] =  0                       - (p->lqr_data.body_theta      );
    delta[9] =  0                       - (p->lqr_data.body_dot_theta  );

//离地保护相关代码
    if(off_ground_flag_ == true) {
        delta[0] = 0;
        delta[1] = 0;
        delta[2] = 0;
        delta[2] = 0;
        delta[3] = 0;
        delta[4] = 0;
        delta[5] = 0;
        delta[6] = 0;
        delta[7] = 0;
        delta[8] = 0;
        delta[9] = 0;
        robot_snap->snap_clear_S();
        robot_snap->snap_set_zero();
    }

    controller_.lqr_controller->fit_clc(delta,p->left_leg.L0,p->right_leg.L0);

    auto robot_ctrl = &controller_;
    auto left =  controller_.lqr_controller->get_lqr_output(LQR::E_left);
    auto right = controller_.lqr_controller->get_lqr_output(LQR::E_right);

    robot_ctrl->left_vmc_pkg.force_y = LEG_FORWARD(mode_state_.height_record);
    robot_ctrl->left_vmc_pkg.force_x = 0;
    robot_ctrl->right_vmc_pkg.force_y = LEG_FORWARD(mode_state_.height_record);
    robot_ctrl->right_vmc_pkg.force_x = 0;

    robot_ctrl->left_vmc_pkg.force_L = controller_.leg_ctrl->get_output().force_left;
    robot_ctrl->left_vmc_pkg.leg_tor = left.body_balance+left.body_move;
    robot_ctrl->right_vmc_pkg.force_L = controller_.leg_ctrl->get_output().force_right;
    robot_ctrl->right_vmc_pkg.leg_tor = right.body_balance+right.body_move;

    robot_ctrl->vmc->tor_clc(robot_ctrl->left_vmc_pkg,p->left_leg,VMC::E_Left);
    robot_ctrl->vmc->tor_clc(robot_ctrl->right_vmc_pkg,p->right_leg, VMC::E_Right);

    //更新到目标输出中
    auto answer= robot_ctrl->vmc->tor_get();
    motor_output_.tor_j1 = answer.p_right_tor2 + answer.c_right_tor2;
    motor_output_.tor_j2 = answer.p_right_tor1 + answer.c_right_tor1;
    motor_output_.tor_j3 = answer.p_left_tor1 + answer.c_left_tor1;
    motor_output_.tor_j4 = answer.p_left_tor2 + answer.c_left_tor2;

    motor_output_.dynamic_left = left.wheel_balance + left.wheel_move;
    motor_output_.dynamic_right = right.wheel_balance + right.wheel_move;
}

//土狗模式部分
void app_coordinate::exe_dog(snap *robot_snap, mode_state_struct state, ctrl_struct ctrl){
    // if(state.last_state != E_DOG && state.current_state_ == E_DOG) {
    //     controller_.dog_ctrl->clear();
    // }
    auto p = robot_snap->current_snap_get();
    //分解合成目标速度
    //当角度误差较小的时候进行（小于30°）的时候进行cos计算

    //step1: 计算目标速度角度和大小
    float ver = ctrl.ver_y;
    float gry = ctrl.gry;

    controller_.dog_ctrl->speed_update(ver,gry,
        p->robot_raw_data.speed_left,p->robot_raw_data.speed_right);
    //step2: 更新数据
    // if(abs(deg - p->robot_raw_data.body_phi) < PI/6) {
    //     controller_.dog_ctrl->speed_update(ver*cos(deg - p->robot_raw_data.body_phi),
    //         p->robot_raw_data.speed_left,p->robot_raw_data.speed_right,
    //         p->robot_raw_data.body_phi,p->robot_raw_data.dot_phi);
    // }
    // else {
    //     controller_.dog_ctrl->speed_update(0,
    //         p->robot_raw_data.speed_left,p->robot_raw_data.speed_right,
    //         p->robot_raw_data.body_phi,p->robot_raw_data.dot_phi);
    // }
    //更新腿部控制

    //step3: 拉取数据并且输出
    motor_output_.tor_j1 = 0;
    motor_output_.tor_j2 = 0;
    motor_output_.tor_j3 = 0;
    motor_output_.tor_j4 = 0;

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

void app_coordinate::ibc_send_update(float vector_x, float vector_y,
    float vector_z, mode_state mode){

}
