//
// Created by 15082 on 2026/3/28.
//

#include "app_gimbal_coordinate.h"

#include "robomaster.h"
#include "robot_data.h"

PID yaw_pos(25, 0, 0, 15, 0);
PID yaw_speed(2, 0.1 / 1000.0f, 0, 3, 0.2);

PID fric_speed(10, 1, 0, 16384, 3000);
PID trigger_speed(20, 0, 0, 16384, 10000);


using namespace Gimbal;

void gimbal_coordinate::tick() {
    auto cmd = ctrl_.get_cmd();
    auto flag = ctrl_.get_flag();
    auto vision = vision_->get_target();

    if(flag->auto_aim_gimbal_ == true && flag->gimbal_ctrl_ == false) {
        //此处需要进行二次轨迹规划
        if(vision.target_pitch != 0 || vision.target_yaw != 0) {
            target_pit_ = pitch_path.update(vision.target_pitch);
            auto raw_yaw      = vision.target_yaw;
            target_yaw_ = yaw_path.update_limit(raw_yaw);
        }
        head_->head_pid_clc(target_yaw_,target_pit_);
        head_->head_output();
    }
    else if(flag->auto_aim_gimbal_ == false && flag->gimbal_ctrl_ == true) {
        raw_pit += cmd->delta_pit/1000;
        raw_pit>0.42?raw_pit=0.42:(raw_pit<-0.315?raw_pit=-0.315:0);
        target_pit_ = pitch_path.update(raw_pit);
        raw_yaw += cmd->delta_head_yaw/1000;
        raw_yaw > PI_F32? raw_yaw -= 2*PI_F32:(raw_yaw < -PI_F32?raw_yaw+= 2*PI_F32:0);
        target_yaw_ = yaw_path.update_limit(raw_yaw);
        head_->head_pid_clc(target_yaw_,target_pit_);
        head_->head_output();
    }
    else {
        head_->head_relax();
        pitch_path.clear();
        yaw_path.clear();
    }

    if(flag->auto_aim_shoot_ == true && flag->shoot_ctrl_ == true) {
        if(vision.fire_ctrl_cmd == 1)
            shoot_->shoot_update(Gimbal::E_FRIC_FAST, Gimbal::E_TRIGGER_REST);
        else if(vision.fire_ctrl_cmd == 2)
            shoot_->shoot_update(Gimbal::E_FRIC_FAST, Gimbal::E_TRIGGER_FAST);
        else
            shoot_->shoot_update(Gimbal::E_FRIC_REST, Gimbal::E_TRIGGER_REST);
    }
    else if(flag->auto_aim_shoot_ == false && flag->shoot_ctrl_ == true)
        shoot_->shoot_update(Gimbal::E_FRIC_FAST, Gimbal::E_TRIGGER_FAST);
    else
        shoot_->shoot_update(E_FRIC_REST,E_TRIGGER_REST);

    shoot_->shoot_tick();
}

void gimbal_coordinate::init() {
    vision_->init();

    ctrl_.init();
    head_->head_init(yaw_pos,yaw_speed);
    shoot_->shoot_init(fric_speed,trigger_speed);
}

void gimbal_coordinate::update_rc(const bsp_rc_data_t *rc) {
    ctrl_.rc_update(rc);
}

void gimbal_coordinate::update_keyboard(Gimbal_cmd::keybroad_pkg pkg_) {
    ctrl_.keyboard_update(pkg_);
}

void gimbal_coordinate::update_video(){

}

