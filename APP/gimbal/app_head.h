//
// Created by 15082 on 2026/2/4.
//

#ifndef APP_HEAD_H
#define APP_HEAD_H
#include "app_midaverage_filter.h"
#include "app_motor.h"
#include "ctrl_pid.h"
#include "dev_motor_dji.h"
#include "dev_motor_dm.h"
#define GM6020_TOR_TO_CURRENT(tor) (tor/0.741f)

namespace Gimbal {
typedef struct {
    float yaw_zero_point;
    float yaw_pos;
    float yaw_speed;
    float yaw_current;
    float pit_pos;
    float pit_speed;
}head_pkg;

class Head {
public:
    Head(Motor::DJIMotor *yaw_ptr, Motor::DMMotor *pitch_ptr)
        :yaw_motor_(yaw_ptr), pit_motor_(pitch_ptr) {

    }
    void head_init(const Controller::PID& yaw_pos_param,
        const Controller::PID& yaw_speed_param,
         const Controller::PID& pit_pos_param,
         const Controller::PID& pit_speed_param
        );
    void head_pid_clc(float delta_yaw, float delta_pit);
    void head_relax();
    void head_update();
    void head_active();
    void head_output();
    head_pkg gimbal_pkg_ = {};
    float pid_pit_out_ = 0, pid_yaw_out_ = 0; //pit的输出值为扭矩单位Nm，yaw的输出为电压，单位为百分比-1~+1

private:
    Motor::DJIMotor *yaw_motor_;
    Motor::DMMotor *pit_motor_;
    Controller::PID pit_speed_, pit_pos_, yaw_speed_, yaw_pos_;
    TrimmedMeanFilter<10,float> yaw_filter_;
    uint16_t head_flag_{};
};
}




#endif //APP_HEAD_H
