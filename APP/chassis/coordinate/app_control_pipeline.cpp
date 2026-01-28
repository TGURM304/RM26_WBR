//
// Created by 15082 on 2026/1/27.
//

#include "app_control_pipeline.h"

using namespace Pipeline;
void Pipeline::control_pipeline::observer_update() {
    adapter_->update();//更新观测器数据
    lqr_data_ = adapter_->get_LQR_raw();
    left_leg_ = adapter_->get_left_leg_status();
    right_leg_ = adapter_->get_right_leg_status();
}

void control_pipeline::data_clear() {
    adapter_->clear();//清除观测器数据
    lqr_data_ = {};
    left_leg_ = {};
    right_leg_ = {};
    leg_pid_output_ = {};
    leg_controller_->leg_clear();//清除腿长PID
    vmc_tor_ = {};
    memset(lqr_delta_state_,0,sizeof(lqr_delta_state_)); //清除LQR状态差值
    adapter_->clear();
    output_ = {}; //清除总输出
    ver_switch_ = {}; //清除速度控制开关
    lqr_target_ = {}; //清除LQR目标状态
}

void control_pipeline::rest(){
    control_state_ = CHASSIS_REST;
}

void control_pipeline::lqr_control(float32_t target_ver, float32_t phi_gry){
    //计算delta
    //需要把控制系统从位控到速控的转折点：
    // 1. 打滑时
    // 2. delta_S过大时候（顶着墙，速度过大产生误差积累）
    // 定义牵引里程delta_S，当我们的牵引里程S大于某个阈值，我们认为进入顶墙状态
    // 定义delta_ver阈值（是单轮子），当我们的目标速度误差大于某个阈值，我们认为进入打滑状态

    //更新速度控制开关
    if(control_state_ & SLIP_ENABLE == 0) {
        ver_switch_.slip_cnt = 0;
        ver_switch_.delta_S = 0;
    }
    else if(control_state_ & SLIP_ENABLE) {
        if((ABS(lqr_data_.dot_S - target_ver) > DELTA_VER_EDGE || ABS(ver_switch_.delta_S) > DELTA_S_EDGE)
            && ver_switch_.slip_cnt < SLIP_CNT_EDGE_HIGH)
            ver_switch_.slip_cnt += 1, ver_switch_.delta_S += (lqr_data_.dot_S - target_ver)/1000;
        else if(ABS(lqr_data_.dot_S - target_ver) <= DELTA_VER_EDGE && ver_switch_.slip_cnt > 0)
            ver_switch_.slip_cnt -= 1;
        if(ver_switch_.slip_cnt == 0)
            ver_switch_.delta_S = 0;
    }

    //更新目标值
    if(ver_switch_.slip_cnt >= SLIP_CNT_EDGE_LOW) {
        //此处的衰减系数可以调节，公式为：k=h^(1/n)，n为衰减次数，k为每次衰减系数，h为最终衰减到的值
        //此处我们取h = 0.3， n = 300
        lqr_target_.S *= 0.996;
        lqr_target_.dot_S = target_ver;
    }
    else if(ver_switch_.slip_cnt < SLIP_CNT_EDGE_LOW) {
       lqr_target_.S += target_ver * 0.001f;
       lqr_target_.dot_S = target_ver;
    }
    lqr_target_.phi += phi_gry * 0.001f;
    lqr_target_.dot_phi = phi_gry;
    lqr_target_.left_theta = lqr_target_.left_dot_theta = 0.0f;
    lqr_target_.right_theta = lqr_target_.right_dot_theta = 0.0f;
    lqr_target_.body_theta = lqr_target_.body_dot_theta = 0.0f;
    //更新delta_state
    lqr_delta_state_[0] = lqr_target_.S - lqr_data_.S;
    lqr_delta_state_[1] = lqr_target_.dot_S - lqr_data_.dot_S;
    lqr_delta_state_[2] = lqr_target_.phi - lqr_data_.phi;
    lqr_delta_state_[3] = lqr_target_.dot_phi - lqr_data_.dot_phi;
    lqr_delta_state_[4] = lqr_target_.left_theta - lqr_data_.left_theta;
    lqr_delta_state_[5] = lqr_target_.left_dot_theta - lqr_data_.left_dot_theta;
    lqr_delta_state_[6] = lqr_target_.right_theta - lqr_data_.right_theta;
    lqr_delta_state_[7] = lqr_target_.right_dot_theta - lqr_data_.right_dot_theta;
    lqr_delta_state_[8] = lqr_target_.body_theta - lqr_data_.body_theta;
    lqr_delta_state_[9] = lqr_target_.body_dot_theta - lqr_data_.body_dot_theta;
    //计算LQR结果
    if(control_state_ & CHASSIS_STATIC_LQR)
        lqr_->static_clc(lqr_delta_state_);
    else if(control_state_ & CHASSIS_DYNAMIC_LQR)
        lqr_->dynamic_clc(lqr_delta_state_, left_leg_,right_leg_);
}

void control_pipeline::lqr_get(){
    auto p =lqr_->get_out_tor();
    lqr_out_tor_.T_left_W = p[0];
    lqr_out_tor_.T_right_W = p[1];
    lqr_out_tor_.T_left_B = p[2];
    lqr_out_tor_.T_right_B = p[3];
}

void control_pipeline::leg_len_control(float32_t left_len, float32_t right_len) {
    if(control_state_ & LEG_LEN_ENABLE) {
        leg_controller_->left_len_update(left_leg_,left_len);
        leg_controller_->right_len_update(right_leg_,right_len);
        leg_pid_output_ = leg_controller_->get_output();
    }
    else if(control_state_ & LEG_LEN_ENABLE == 0) {
        leg_controller_->leg_clear();
        leg_pid_output_ = {};
    }
}

void control_pipeline::vmc_pkg_update(){
    ctrl_pkg_left_.force_L = leg_pid_output_.force_left + FORWARD_FEED;
    ctrl_pkg_right_.force_L = leg_pid_output_.force_right+FORWARD_FEED;
    ctrl_pkg_left_.leg_tor = lqr_out_tor_.T_left_W;
    ctrl_pkg_right_.leg_tor = lqr_out_tor_.T_right_W;
}

void control_pipeline::vmc_clc(){
    vmc_->tor_clc(ctrl_pkg_left_,left_leg_,VMC::E_Left);
    vmc_->tor_clc(ctrl_pkg_right_,right_leg_,VMC::E_Right);
    vmc_tor_ = vmc_->tor_get();
}

void control_pipeline::set_state(ctrl_cmd lqr_cmd,
    ctrl_cmd leg_pid_cmd, ctrl_cmd slip_cmd, ctrl_cmd safe_cmd) {
    if(lqr_cmd == E_LQR_STATIC)
        control_state_ |= CHASSIS_STATIC_LQR, control_state_ &= ~CHASSIS_DYNAMIC_LQR;
    else if(lqr_cmd == E_LQR_DYNAMIC)
        control_state_ |= CHASSIS_DYNAMIC_LQR, control_state_ &= ~CHASSIS_STATIC_LQR;
    else if(lqr_cmd == E_LQR_JUMP) {
        control_state_ |= CHASSIS_JUMP;
        control_state_ &= ~CHASSIS_DYNAMIC_LQR;
        control_state_ &= ~CHASSIS_STATIC_LQR;
    }
    if(leg_pid_cmd == E_LEG_PID_ENABLE)
        control_state_ |= LEG_LEN_ENABLE;
    else if(leg_pid_cmd == E_LEG_PID_DISABLE)
        control_state_ &= ~LEG_LEN_ENABLE;
    if(slip_cmd == E_SLIP_ENABLE)
        control_state_ |= SLIP_ENABLE;
    else if(slip_cmd == E_SLIP_DISABLE) control_state_ &= ~SLIP_ENABLE;
    if(safe_cmd == E_CHASSIS_SAFE)
        control_state_ |= CHASSIS_SAFE;
    else if(safe_cmd == E_CHASSIS_USING)
        control_state_ &= ~CHASSIS_SAFE;
}

void control_pipeline::motor_tor_update(){
    output_.tor1 = vmc_tor_.p_right_tor2;
    output_.tor2 = vmc_tor_.p_right_tor1;
    output_.tor3 = vmc_tor_.p_left_tor1;
    output_.tor4 = vmc_tor_.p_left_tor2;
}
