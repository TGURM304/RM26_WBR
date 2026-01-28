//
// Created by 15082 on 2026/1/16.
//

#include "app_motor_pkg.h"

void Motor_Pkg::Joint::set_tor(float tor) {
    float temp = tor;
    if(tor > 40) temp = 40;
    else if(tor < -40) temp = -40;
    control(0,0,0,0,temp*dir_);
}


Motor_Pkg::motor_status_pkg Motor_Pkg::Joint::get_status() {
    status_pkg_.old_pos = status_pkg_.pos;
    status_pkg_.old_speed = status_pkg_.speed;
    status_pkg_.pos = status.pos*dir_+zero_;
    status_pkg_.speed = status.vel*dir_;
    status_pkg_.err = status.err;
    return status_pkg_;
}

void Motor_Pkg::Dynamic::pkg_init() {
    init();
}
/*
 * 铁损阻力：72rpm/Nm
 * 减速比原装：3591/187
 * 减速比：268/17
 * 扭矩常数：0.3Nm/A
 * 机械转角：0~8192
 * 转子转速单位：RPM
 * 电流： -20A~20A  -16384~16384
 */
#define REDUCTION_ORG (3591.0f/187.0f)
#define REDUCTION_NOW (268.0f/17.0f)
#define TORQUE_CONST (0.3f)
#define PI_F32 (3.1415926f)
void Motor_Pkg::Dynamic::set_tor(float tor) {
    float temp_tor = tor*dir_;
    // float current = temp_tor/(REDUCTION_NOW/REDUCTION_ORG*TORQUE_CONST);
    float current = temp_tor/REDUCTION_NOW*(REDUCTION_ORG)/TORQUE_CONST;
    if(current > 20.0f) current = 20.0f;
    if(current < -20.0f) current = -20.0f;
    update(current/20*16384);
}

void Motor_Pkg::Dynamic::rest() {
    update(0);
}

Motor_Pkg::motor_status_pkg Motor_Pkg::Dynamic::get_status() {
    old_encode_ = encode_;
    status_.old_pos = status_.pos;
    status_.old_speed = status_.speed;

    encode_ = status.angle;
    float delta_encode = (encode_ - old_encode_)*dir_;
    if(abs(encode_ - old_encode_) > 4096.0f) {
        delta_encode >0 ? delta_encode -= 8192.0f : delta_encode += 8192.0f;
    }
    status_.pos += delta_encode/(8192.0f)*2.0f*PI_F32/(REDUCTION_NOW)*wheel_R_;
    status_.speed = status.speed*dir_*2.0f*PI_F32/60.0f/(REDUCTION_NOW)*wheel_R_;
    return status_;
}

void Motor_Pkg::Dynamic::status_clear() {
    status_.old_pos = 0.0f;
    status_.old_speed = 0.0f;
    status_.pos = 0.0f;
    status_.speed = 0.0f;
    encode_ = status.angle;
    old_encode_ = encode_;
}