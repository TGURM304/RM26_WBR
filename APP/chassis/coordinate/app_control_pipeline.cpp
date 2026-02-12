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
    output_ = {}; //清除总输出
    ver_switch_ = {}; //清除速度控制开关
    lqr_target_ = {}; //清除LQR目标状态
}

void control_pipeline::rest(){
    control_state_ = CHASSIS_REST;
}

void control_pipeline::lqr_control(float32_t target_ver, float32_t phi_gry){
    //此函数仅处理LQR计算相关内容
    if(control_state_ & CHASSIS_SAFE) {
        lqr_delta_state_[0] = 0;
        lqr_delta_state_[1] = 0;
        lqr_delta_state_[2] = 0;
        lqr_delta_state_[3] = 0;
        lqr_delta_state_[4] = 0;
        lqr_delta_state_[5] = 0;
        lqr_delta_state_[6] = 0;
        lqr_delta_state_[7] = 0;
        lqr_delta_state_[8] = 0;
        lqr_delta_state_[9] = 0;
        lqr_->static_clc(lqr_delta_state_);
        lqr_target_.phi = lqr_data_.phi;
        return;
    }
    //目标速度
    lqr_target_.S += target_ver/1000.0f;
    lqr_target_.dot_S = target_ver;
    //此处需要进行过零处理
    lqr_target_.phi += phi_gry/1000.0f;
    if(lqr_target_.phi > PI) lqr_target_.phi -= 2*PI;
    else if(lqr_target_.phi < -PI) lqr_target_.phi += 2*PI;
    lqr_target_.dot_phi = phi_gry;
    //目标的平衡摆角一直都是0
    lqr_target_.left_theta = lqr_target_.left_dot_theta = 0.0f;
    lqr_target_.right_theta = lqr_target_.right_dot_theta = 0.0f;
    lqr_target_.body_theta = lqr_target_.body_dot_theta = 0.0f;

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
    if(control_state_ & SLIP_ENABLE) {
        if(ver_switch_.att_start_flag == 1) {
            //此处的衰减系数可以调节，公式为：k=h^(1/n)，n为衰减次数，k为每次衰减系数，h为最终衰减到的值
            //此处我们取h = 0.3， n = 300， k= 0.996
            ver_switch_.att_S_temp *= 0.996;
        }
        if(ABS(lqr_target_.S - lqr_data_.S) > DELTA_S_EDGE ||
            ABS(lqr_target_.dot_S - lqr_data_.dot_S) > DELTA_VER_EDGE) {
                if(ver_switch_.att_start_flag == 1)
                    ver_switch_.att_cnt = (ver_switch_.att_cnt+10)>500?500:ver_switch_.att_cnt+10;
                else if(ver_switch_.att_start_flag == 0) {
                    ver_switch_.start_cnt += 1;
                    if(ver_switch_.start_cnt >= 20) {
                        ver_switch_.start_cnt = 0;
                        ver_switch_.att_start_flag = 1;
                        ver_switch_.att_cnt = 500;
                        ver_switch_.att_S_temp = lqr_target_.S - lqr_data_.S;
                    }
                }
            }
        else if(ABS(lqr_target_.dot_S - lqr_data_.dot_S) < DELTA_VER_EDGE) {
            ver_switch_.att_cnt-=1;
            if(ver_switch_.att_cnt <= 0)
                ver_switch_.att_start_flag = 0, ver_switch_.att_cnt = 0;
        }
    }

    //更新delta_state
    // lqr_delta_state_[0] = lqr_target_.S - lqr_data_.S;
    // //如果启用了衰减保护系统就引入保护
    // if((control_state_ & SLIP_ENABLE) == SLIP_ENABLE &&
    //     ver_switch_.att_start_flag == 1)
    //     lqr_delta_state_[0] = ver_switch_.att_S_temp;
    // lqr_delta_state_[1] = DATA_RANGE(lqr_target_.dot_S - lqr_data_.dot_S,3);
    // lqr_delta_state_[2] = lqr_target_.phi - lqr_data_.phi;
    // lqr_delta_state_[3] = lqr_target_.dot_phi - lqr_data_.dot_phi;
    // lqr_delta_state_[4] = lqr_target_.left_theta - lqr_data_.left_theta;
    // lqr_delta_state_[5] = lqr_target_.left_dot_theta - lqr_data_.left_dot_theta;
    // lqr_delta_state_[6] = lqr_target_.right_theta - lqr_data_.right_theta;
    // lqr_delta_state_[7] = lqr_target_.right_dot_theta - lqr_data_.right_dot_theta;
    // lqr_delta_state_[8] = lqr_target_.body_theta - lqr_data_.body_theta;
    // lqr_delta_state_[9] = lqr_target_.body_dot_theta - lqr_data_.body_dot_theta;

    if(control_state_ & CHASSIA_CHAIR) {
        lqr_delta_state_[0] = lqr_target_.S - lqr_data_.S;
        lqr_delta_state_[1] = lqr_target_.dot_S - lqr_data_.dot_S;
        lqr_delta_state_[2] = lqr_target_.phi - lqr_data_.phi;
        if(ABS(lqr_delta_state_[2]) > PI)
            lqr_delta_state_[2] -= ABS(lqr_delta_state_[2])/lqr_delta_state_[2]*2*PI;
        lqr_delta_state_[3] = lqr_target_.dot_phi - lqr_data_.dot_phi;
        lqr_delta_state_[4] = 0;
        lqr_delta_state_[5] = 0;
        lqr_delta_state_[6] = 0;
        lqr_delta_state_[7] = 0;
        lqr_delta_state_[8] = -lqr_data_.body_theta;
        lqr_delta_state_[9] = -lqr_data_.body_dot_theta;
    }

 
    //计算LQR结果

    lqr_->static_clc(lqr_delta_state_);
    if((control_state_ & CHASSIS_STATIC_LQR))
        lqr_->static_clc(lqr_delta_state_);
    else if(control_state_ & CHASSIS_DYNAMIC_LQR)
        lqr_->dynamic_clc(lqr_delta_state_, left_leg_,right_leg_);
}

void control_pipeline::lqr_get(){
    lqr_left = lqr_->get_lqr_output(LQR::E_left);
    lqr_right  = lqr_->get_lqr_output(LQR::E_right);
}

void control_pipeline::leg_len_control(float32_t left_len, float32_t right_len) {
    if(control_state_ & LEG_LEN_ENABLE) {
        leg_controller_->left_len_update(left_leg_,left_len);
        leg_controller_->right_len_update(right_leg_,right_len);
        leg_pid_output_ = leg_controller_->get_output();
    }
    else if((control_state_ & LEG_LEN_ENABLE) == 0) {
        leg_controller_->leg_len_clear();
    }
}

void control_pipeline::leg_deg_control(float32_t left_deg, float32_t right_deg){
    if(control_state_ & LEG_DEG_ENABLE) {
        leg_controller_->left_deg_update(left_leg_,left_deg);
        leg_controller_->right_deg_update(right_leg_,right_deg);
    }
    else if((control_state_ & LEG_DEG_ENABLE) == 0) {
        leg_controller_->leg_deg_clear();
    }
}

void control_pipeline::vmc_pkg_update(){
    ctrl_pkg_left_.leg_tor = 0;
    ctrl_pkg_right_.leg_tor = 0;
    ctrl_pkg_left_.force_L = 0;
    ctrl_pkg_right_.force_L = 0;
    ctrl_pkg_left_.force_y = 0;
    ctrl_pkg_left_.force_x = 0;
    ctrl_pkg_right_.force_y = 0;
    ctrl_pkg_right_.force_x = 0;


    if(control_state_ & LEG_LEN_ENABLE) {
        ctrl_pkg_left_.force_L = leg_controller_->get_output().force_left;
        ctrl_pkg_right_.force_L = leg_controller_->get_output().force_right;
    }
    if(control_state_ & LEG_DEG_ENABLE) {
        ctrl_pkg_left_.leg_tor = leg_controller_->get_output().tor_left;
        ctrl_pkg_right_.leg_tor = leg_controller_->get_output().tor_right;
    }
    if(control_state_ & LEG_FORWARD_ENABLE) {
        ctrl_pkg_left_.force_y = 40*cos(lqr_data_.body_theta);
        ctrl_pkg_left_.force_x = -40*sin(lqr_data_.body_theta);
        ctrl_pkg_right_.force_y = 40*cos(lqr_data_.body_theta);
        ctrl_pkg_right_.force_x = -40*sin(lqr_data_.body_theta);
    }
}

void control_pipeline::vmc_clc(){
    vmc_->tor_clc(ctrl_pkg_left_,left_leg_,VMC::E_Left);
    vmc_->tor_clc(ctrl_pkg_right_,right_leg_,VMC::E_Right);
    vmc_tor_ = vmc_->tor_get();
}

void control_pipeline::set_state(cmd_pkg my_cmd_pkg) {
    if(my_cmd_pkg.lqr_cmd == E_LQR_STATIC) {
        control_state_ &= ~CHASSIA_CHAIR;
        control_state_ &= ~CHASSIS_JUMP;
        control_state_ &= ~CHASSIS_DYNAMIC_LQR;
        control_state_ |= CHASSIS_STATIC_LQR;
    }else if(my_cmd_pkg.lqr_cmd == E_LQR_DYNAMIC) {
        control_state_ &= ~CHASSIA_CHAIR;
        control_state_ &= ~CHASSIS_JUMP;
        control_state_ |= CHASSIS_DYNAMIC_LQR;
        control_state_ &= ~CHASSIS_STATIC_LQR;
    }else if(my_cmd_pkg.lqr_cmd == E_LQR_JUMP) {
        control_state_ &= ~CHASSIA_CHAIR;
        control_state_ |= CHASSIS_JUMP;
        control_state_ &= ~CHASSIS_DYNAMIC_LQR;
        control_state_ &= ~CHASSIS_STATIC_LQR;
    }
    else if(my_cmd_pkg.lqr_cmd == E_LQR_CHAIR) {
        control_state_ |= CHASSIA_CHAIR;
        control_state_ &= ~CHASSIS_JUMP;
        control_state_ &= ~CHASSIS_DYNAMIC_LQR;
        control_state_ &= ~CHASSIS_STATIC_LQR;
    }

    if(my_cmd_pkg.leg_len_cmd == E_LEG_LEN_ENABLE)
        control_state_ |= LEG_LEN_ENABLE;
    else if(my_cmd_pkg.leg_len_cmd == E_LEG_LEN_DISABLE)
        control_state_ &= ~LEG_LEN_ENABLE;
    if(my_cmd_pkg.leg_deg_cmd == E_LEG_DEG_ENABLE)
        control_state_ |= LEG_DEG_ENABLE;
    else if(my_cmd_pkg.leg_deg_cmd == E_LEG_DEG_DISABLE)
        control_state_ &= ~LEG_DEG_ENABLE;
    if(my_cmd_pkg.leg_forward_cmd == E_LEG_FORWARD_ENABLE)
        control_state_ |= LEG_FORWARD_ENABLE;
    else if(my_cmd_pkg.leg_forward_cmd == E_LEG_FORWARD_DISABLE)
        control_state_ &= ~LEG_FORWARD_ENABLE;

    if(my_cmd_pkg.slip_cmd == E_SLIP_ENABLE)
        control_state_ |= SLIP_ENABLE;
    else if(my_cmd_pkg.slip_cmd == E_SLIP_DISABLE) control_state_ &= ~SLIP_ENABLE;

    if(my_cmd_pkg.safe_cmd == E_CHASSIS_SAFE)
        control_state_ |= CHASSIS_SAFE;
    else if(my_cmd_pkg.safe_cmd == E_CHASSIS_USING)
        control_state_ &= ~CHASSIS_SAFE;
}

void control_pipeline::motor_tor_update(){
    //此处tor1 tor2的VMC定义和编号定义不太一样，不要弄混了
    output_.tor1 = vmc_tor_.p_right_tor2+vmc_tor_.c_right_tor2;
    output_.tor2 = vmc_tor_.p_right_tor1+vmc_tor_.c_right_tor1;
    output_.tor3 = vmc_tor_.p_left_tor1+vmc_tor_.c_left_tor1;
    output_.tor4 = vmc_tor_.p_left_tor2+vmc_tor_.c_left_tor2;
    output_.dynamic_left = lqr_left.wheel_balance+lqr_left.wheel_move;
    output_.dynamic_right = lqr_right.wheel_balance+lqr_right.wheel_move;
}
