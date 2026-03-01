//
// Created by 15082 on 2026/3/1.
//

#ifndef APP_BIQUAD_FILTER_H
#define APP_BIQUAD_FILTER_H

namespace Filter {
    typedef enum {
        E_HIGH_PASS,
        E_LOW_PASS
    }filter_type;
    class BiquadFilter {
    public:
        BiquadFilter() = default;
        BiquadFilter(float fc, float fs,filter_type type);//fc为截止频率，fs为采样频率
        float process(float input);
        void clear();
    private:
        float b0_ = 0, b1_ = 0, b2_ = 0, a1_ = 0, a2_ = 0; // 滤波器系数
        float x1_ = 0.0f, x2_ = 0.0f; // 输入历史
        float y1_ = 0.0f, y2_ = 0.0f; // 输出历史
    };
    class band_resistor {
    public:
      band_resistor() = default;
        band_resistor(float fc_low, float fc_high, float fs, float order);//fc_low为低截止频率，fc_high为高截止频率，fs为采样频率，order为滤波器阶数,必须为偶数,最大为4
        float process(float input);
        void clear();
    private:
        BiquadFilter low_pass_filters_[2];
        BiquadFilter high_pass_filters_[2];
        float low_pass_out = 0, high_pass_out = 0, band_pass_out = 0;
        float order_ = 0;
    };
}

#endif //APP_BIQUAD_FILTER_H
