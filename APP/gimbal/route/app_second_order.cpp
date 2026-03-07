//
// Created by 15082 on 2026/3/8.
//

#include "app_second_order.h"

using namespace PathReference;

void second_order::clear() {
    x1_ = 0;
    x2_ = 0;
}

void second_order::param_set(float update_freq, float omega_n, float epsilon) {
    T_ = 1/update_freq;
    omega_n_ = omega_n;
    epsilon_ = epsilon;
}

float second_order::update(float target) {
    float x1_dot = x2_;
    float x2_dot = -2*epsilon_*omega_n_*x2_ - omega_n_*omega_n_*x1_ + omega_n_*omega_n_*target;
    x1_ += x1_dot*T_;
    x2_ += x2_dot*T_;
    return x1_;
}