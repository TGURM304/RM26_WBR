//
// Created by 15082 on 2026/3/9.
//

#ifndef APP_GIMBAL_SNAP_H
#define APP_GIMBAL_SNAP_H
#include "app_ins.h"
#include "dev_motor_dji.h"
#include "dev_motor_dm.h"
#include "app_ibc.h"

#define PITCH_ZERO_POINT 1.12f
#define PIT_LIMIT_MAX 1.5
#define PIT_LIMIT_MIN 0.75
namespace Gimbal {
//云台快照系统，所有的控制器统一接口，方便后续的功能开发和维护
typedef struct {
    float ins_pit, ins_yaw, ins_rol;
    float ins_pit_dot, ins_yaw_dot, ins_rol_dot;
    float yaw_motor_encoder, pit_motor_encoder;
    float pit_motor_ver, yaw_motor_ver;
    float trigger_speed;
    float fric_left_speed, fric_right_speed;
}snap_pkg;
class snap {
public:
    snap() = default;
    snap(Motor::DJIMotor *yaw_ptr,
        Motor::DMMotor *pitch_ptr,
        Motor::DJIMotor *trigger_ptr,
        Motor::DJIMotor *fric_left_ptr,
        Motor::DJIMotor *fric_right_ptr,
        const app_ins_data_t *ins,
        float pitch_zero_point,
        float yaw_zero_point)
    : yaw_ptr_(yaw_ptr), pitch_ptr_(pitch_ptr), ins_(ins),
        pitch_zero_point_(pitch_zero_point), yaw_zero_point_(yaw_zero_point),
        trigger_ptr_(trigger_ptr),fric_left_ptr_(fric_left_ptr),
        fric_right_ptr_(fric_right_ptr), ibc_chassis(E_CAN3,CHASSIS_ID) {
    }
    void snap_update();
    snap_pkg get_snap_pkg();
    void snap_clear();
    void snap_init() {
        ibc_chassis.init();
    }
    [[nodiscard]] float get_yaw_zero() const;
    [[nodiscard]] float get_pitch_zero() const;
private:
    float pitch_zero_point_ = 0, yaw_zero_point_ = 0;
    snap_pkg snap_pkg_ = {};
    Motor::DJIMotor *yaw_ptr_ = nullptr;
    Motor::DMMotor *pitch_ptr_ = nullptr;
    Motor::DJIMotor *trigger_ptr_ = nullptr;
    Motor::DJIMotor *fric_left_ptr_ = nullptr;
    Motor::DJIMotor *fric_right_ptr_ = nullptr;
    const app_ins_data_t *ins_ = nullptr;
    IBC::gimbal gimbal_send_ = {};
    app_msg_can_receiver<IBC::chassis> ibc_chassis;

};
}



#endif //APP_GIMBAL_SNAP_H
