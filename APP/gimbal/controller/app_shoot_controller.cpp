//
// Created by 15082 on 2026/3/8.
//

#include "app_shoot_controller.h"

void Gimbal::Shoot::shoot_update(fric_mode_e fric_mode, trigger_mode_e trigger_mode) {
    //根据状态机更新射击电机的速度和触发电机的状态
    shoot_state_.fric_flag = fric_mode;
    shoot_state_.trigger_flag = trigger_mode;

    if(fric_mode == E_FRIC_REST)
        fric_target_speed_ = fric_left_out_ = fric_right_out_ = 0;
    else if(fric_mode == E_FRIC_SLOW)
        fric_target_speed_ = 3000;
    else if(fric_mode == E_FRIC_FAST)
        fric_target_speed_ = 5000;

    if(trigger_mode == E_TRIGGER_REST)
        trigger_out_ = trigger_target_speed_ = 0;
    else if(trigger_mode == E_TRIGGER_SLOW)
        trigger_target_speed_ = -1000;
    else if(trigger_mode == E_TRIGGER_FAST)
        trigger_target_speed_ = -2000;
}

void Gimbal::Shoot::shoot_tick() {

    auto snap = snap_ptr_->get_snap_pkg();
    //计算PID输出
    if(shoot_state_.fric_flag == E_FRIC_REST)
        fric_left_out_ = fric_right_out_ = 0;
    else {
        fric_left_out_ =
            fric_left_speed_.update(snap.fric_left_speed, fric_target_speed_);
        fric_right_out_ =
            fric_right_speed_.update(snap.fric_right_speed, -fric_target_speed_);
    }
    if(shoot_state_.trigger_flag == E_TRIGGER_REST)
        trigger_out_ = 0;
    else
        trigger_out_ =
            trigger_out_filter_.process(
                trigger_speed_.update(snap.trigger_speed, trigger_target_speed_));\

    motor_left_->update(fric_left_out_);
    motor_right_->update(fric_right_out_);
    motor_trigger_->update(trigger_out_);

}

void Gimbal::Shoot::shoot_clear() {
    shoot_state_.fric_flag = E_FRIC_REST;
    shoot_state_.trigger_flag = E_TRIGGER_REST;
    fric_left_out_ = 0;
    fric_right_out_ = 0;
    trigger_out_ = 0;
    motor_left_->clear();
    motor_right_->clear();
    motor_trigger_->clear();
}

void Gimbal::Shoot::shoot_param_set(const Controller::PID &fric_speed_param,
    const Controller::PID &trigger_speed_param) {
    fric_left_speed_ = fric_speed_param;
    fric_right_speed_ = fric_speed_param;
    trigger_speed_ = trigger_speed_param;
}