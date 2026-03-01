//
// Created by 15082 on 2026/3/1.
//

#include "app_biquad_filter.h"

#include <math.h>

Filter::BiquadFilter::BiquadFilter(float fc, float fs,filter_type type) {
    //计算滤波器参数
    float Q = 1.0f / sqrtf(2.0f); // Q值，常用的Butterworth滤波器
    float K = tanf(M_PI * fc / fs); // 预计算的中间变量
    float normal = 1.0f / (1.0f + K / Q + K * K); // 归一化系数

    b0_ = K*K*normal;
    if(type == E_HIGH_PASS) {
        b1_ = -2.0f * b0_;
    }
    else if(type == E_LOW_PASS) {
        b1_ = 2.0f * b0_;
    }
    b2_ = b0_;
    a1_ = 2.0f * (K*K - 1.0f) * normal;
    a2_ = (1.0f - K / Q + K * K) * normal;
}

float Filter::BiquadFilter::process(float input){
    // 计算当前输出
    float output = b0_ * input + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;

    // 更新历史输入输出
    x2_ = x1_;
    x1_ = input;
    y2_ = y1_;
    y1_ = output;

    return output;
}

void Filter::BiquadFilter::clear() {
    x1_ = x2_ = y1_ = y2_ = 0.0f;
}

Filter::band_resistor::band_resistor(float fc_low, float fc_high, float fs, float order){
    order_ = order;
    low_pass_filters_[0] = BiquadFilter(fc_low, fs, E_LOW_PASS);
    low_pass_filters_[1] = BiquadFilter(fc_low, fs, E_LOW_PASS);
    high_pass_filters_[0] = BiquadFilter(fc_high, fs, E_HIGH_PASS);
    high_pass_filters_[1] = BiquadFilter(fc_high, fs, E_HIGH_PASS);
}

float Filter::band_resistor::process(float input){
    float temp_low = low_pass_filters_[0].process(input);
    float temp_high = high_pass_filters_[0].process(input);
    if(order_ == 4) {
        low_pass_out = low_pass_filters_[1].process(temp_low);
        high_pass_out = high_pass_filters_[1].process(temp_high);
    }
    else if(order_ == 2) {
        low_pass_out = temp_low;
        high_pass_out = temp_high;
    }
    else {
        low_pass_out = 0;
        high_pass_out = 0;
    }
    band_pass_out = temp_low + temp_high;
    return band_pass_out;
}

void Filter::band_resistor::clear() {
    for(int i = 0; i < 2; i++) {
        low_pass_filters_[i].clear();
        high_pass_filters_[i].clear();
    }
    low_pass_out = high_pass_out = band_pass_out = 0;
}