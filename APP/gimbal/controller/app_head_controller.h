//
// Created by 15082 on 2026/2/4.
//

#ifndef APP_HEAD_H
#define APP_HEAD_H

#include "ctrl_pid.h"
#include "dev_motor_dji.h"
#include "dev_motor_dm.h"
#include "app_biquad_filter.h"
#include "app_gimbal_snap.h"


#define GM6020_TOR_TO_CURRENT(tor) (tor/0.741f)


//该文件代码仅提供云台pitch和yaw的控制器，其他逻辑代码等请另寻他处
namespace Gimbal {
//这里都是目标值和控制量
typedef struct {
    float yaw_pos, pit_pos;
    float yaw_speed;
    float yaw_out;
}ctrl_pkg;
class Head {
public:
    Head(Motor::DJIMotor *yaw_ptr, Motor::DMMotor *pitch_ptr, snap *robo_snap)
        :yaw_motor_(yaw_ptr), pit_motor_(pitch_ptr), robo_snap_(robo_snap) {

    }
    void head_init(const Controller::PID& yaw_pos_param,
        const Controller::PID& yaw_speed_param,
        const Controller::PID& pit_pos_param,
        const Controller::PID& pit_speed_param
        );
    void head_pid_clc(float target_yaw, float target_pit);
    void head_relax();
    void head_update();
    void head_active();
    void head_output();
    void head_clear();
    float pid_pit_out_ = 0, pid_yaw_out_ = 0; //pit的输出值为扭矩单位Nm，yaw的输出为电压，单位为百分比-1~+1
private:
    Motor::DJIMotor *yaw_motor_;
    Motor::DMMotor *pit_motor_;
    Controller::PID yaw_speed_, yaw_pos_;
    Controller::PID pit_speed_, pit_pos_;
    Filter::BiquadFilter yaw_out_filter_{ 100, 1000, Filter::E_LOW_PASS };
    Filter::BiquadFilter pit_out_filter_{ 100, 1000, Filter::E_LOW_PASS };
    snap *robo_snap_;
    uint16_t head_flag_{};
    ctrl_pkg head_ctrl_pkg_ = {};
};
}




#endif //APP_HEAD_H
