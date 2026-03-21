//
// Created by 15082 on 2026/3/8.
//

#include "app_second_order.h"

#include "robot_data.h"
#include "dsp/fast_math_functions.h"

using namespace PathReference;

void second_order::clear() {
    process_target_ = 0;
    x1_ = 0;
    x2_ = 0;
}

void second_order::param_set(float update_freq, float omega_n, float epsilon) {
    T_ = 1/update_freq;
    omega_n_ = omega_n;
    epsilon_ = epsilon;
}

float second_order::update(float target) {
    process_target_ = target;
    float x1_dot = x2_;
    float x2_dot = -2*epsilon_*omega_n_*x2_
        - omega_n_*omega_n_*x1_ + omega_n_*omega_n_*process_target_;
    x1_ += x1_dot*T_;
    x2_ += x2_dot*T_;
    return x1_;
}

/**
 * @brief 实现设定当前的跟踪返回值
 * @note 防止轨迹规划问题在初始化的时候回零
 */
void second_order::target_set(float current) {
    process_target_ = current;
    x1_ = current;
    x2_ = 0;
}

/**
 * @brief 处理轨迹规划中跳变问题，直接输入跟踪目标
 *
 * @note 目前仅支持+-PI的对称的区域跳变处理
 */
float second_order::update_limit(float target) {
    float delta   = fmodf(target - process_target_, 2 * PI_F32);
    if(fabsf(delta) > PI_F32)
        delta -= SGN(delta) * 2 * PI_F32;
    process_target_ += delta;
    float out = this->update_mod(process_target_);
    return out;
}

float second_order::update_mod(float target) {
    float t = this->update(target);
    t += PI;
    t = fmodf(t, 2*PI);
    if(t<0)
        t+=2*PI;
    t -= PI;
    return t;
}