//
// Created by 15082 on 2026/1/17.
//

#include "app_WRB_ins.h"
void INS::app_WRB_ins::update() {
    posture_.body_phi      = ins_->yaw/ 180.f * PI_F32;
    posture_.body_phi_gry  = ins_->raw.gyro[2];
    posture_.body_roll     = -ins_->roll / 180.f * PI_F32;
    posture_.body_roll_gry = -ins_->raw.gyro[0];
    posture_.body_theta    = -ins_->pitch / 180.f * PI_F32;
    posture_.body_theta_gry    = -ins_->raw.gyro[1];

    float32_t raw_acc[3];
    raw_acc[0] = ins_->raw.accel[0];
    raw_acc[1] = ins_->raw.accel[1];
    raw_acc[2] = -ins_->raw.accel[2];
    Matrixf<3, 1> raw(raw_acc);
    Matrixf<3, 3> rot    = matrixf::rot_euler<3>(-ins_->roll/180.f*PI_F32,
        -ins_->pitch/180.f*PI_F32, -ins_->yaw/180.f*PI_F32);
    Matrixf<3, 1> answer = rot * raw;
    posture_.body_x_acc  = answer[0][0];
    posture_.body_y_acc  = answer[1][0];
    posture_.body_z_acc  = answer[2][0];
}

const INS::body_posture *INS::app_WRB_ins::get_pos() {
    return &posture_;
}
