//
// Created by 15082 on 2026/3/9.
//

#include "app_gimbal_snap.h"

#include <fast_math_functions.h>

void Gimbal::snap::snap_update() {
    //更新快照数据,其中轮腿的陀螺仪roll和pitch相交换
    snap_pkg_.ins_rol = ins_->pitch/180.0f*PI;
    snap_pkg_.ins_pit = -ins_->roll/180.0f*PI;
    snap_pkg_.ins_yaw = ins_->yaw/180.0f*PI;
    snap_pkg_.ins_rol_dot = ins_->raw.gyro[1];
    snap_pkg_.ins_pit_dot = -ins_->raw.gyro[0];
    snap_pkg_.ins_yaw_dot = ins_->raw.gyro[2];

    snap_pkg_.yaw_motor_encoder = yaw_ptr_->status.angle;
    snap_pkg_.pit_motor_encoder = pitch_ptr_->status.pos;
    snap_pkg_.yaw_motor_ver = -pitch_ptr_->status.vel/60.0f*2*PI;
    snap_pkg_.pit_motor_ver = yaw_ptr_->status.speed/60.0f*2*PI;

    snap_pkg_.trigger_speed = trigger_ptr_->status.speed/60.0f*2*PI;
    snap_pkg_.fric_left_speed = fric_left_ptr_->status.speed/60.0f*2*PI;
    snap_pkg_.fric_right_speed = fric_right_ptr_->status.speed/60.0f*2*PI;
}

Gimbal::snap_pkg Gimbal::snap::get_snap_pkg() {
    return snap_pkg_;
}

void Gimbal::snap::snap_clear() {
    snap_pkg_ = {};
}

float Gimbal::snap::get_yaw_zero() const {
    return yaw_zero_point_;
}

float Gimbal::snap::get_pitch_zero() const {
    return pitch_zero_point_;
}