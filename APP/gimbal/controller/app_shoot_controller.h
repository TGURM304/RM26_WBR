//
// Created by 15082 on 2026/3/8.
//

#ifndef APP_SHOOT_H
#define APP_SHOOT_H
#include "ctrl_pid.h"
#include "dev_motor_dji.h"
#include "app_biquad_filter.h"
#include "app_gimbal_snap.h"

namespace Gimbal {
typedef enum {
    E_TRIGGER_REST = 0,
    E_TRIGGER_SLOW = 1,
    E_TRIGGER_FAST = 2
} trigger_mode_e;
typedef enum {
    E_FRIC_REST = 0,
    E_FRIC_FAST = 1,
} fric_mode_e;
typedef struct {
    fric_mode_e fric_flag = E_FRIC_REST;
    trigger_mode_e trigger_flag = E_TRIGGER_REST;
}shoot_state;
    class Shoot {
    public:
        Shoot() = default;
        Shoot(Motor::DJIMotor *left_shoot, Motor::DJIMotor *right_shoot,
            Motor::DJIMotor *trigger, Gimbal::snap *snap_ptr)
            : motor_left_(left_shoot), motor_right_(right_shoot),
        motor_trigger_(trigger), snap_ptr_(snap_ptr) {
        }
        void shoot_init(
            const Controller::PID& fric_speed_param,
            const Controller::PID& trigger_speed_param);
        void shoot_update(fric_mode_e fric_mode, trigger_mode_e trigger_mode);
        void shoot_tick();//固定周期调用更新
        void shoot_clear();//清空所有内容重置至初始状态
    private:
        Gimbal::snap *snap_ptr_ = nullptr;
        Motor::DJIMotor *motor_left_ = nullptr;
        Motor::DJIMotor *motor_right_ = nullptr;
        Motor::DJIMotor *motor_trigger_ = nullptr;
        Controller::PID fric_left_speed_ = {}, fric_right_speed_ = {},trigger_speed_ = {};
        shoot_state shoot_state_ = {};
        Filter::BiquadFilter trigger_out_filter_{ 100, 1000, Filter::E_LOW_PASS };
        float fric_left_out_ = 0, fric_right_out_ = 0, trigger_out_ = 0;
        float fric_target_speed_ = 0, trigger_target_speed_ = 0;
    };
}



#endif //APP_SHOOT_H
