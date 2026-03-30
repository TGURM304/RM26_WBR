//
// Created by 15082 on 2026/3/13.
//

#include "app_ibc.h"

uint16_t IBC::float32_to_uint16(float32_t data, float data_max, float data_min) {
    float data_range = data_max - data_min;
    float percent = (data - data_min)/data_range;
    percent > 1.0f? percent = 1.0f : (percent < 0.0f? percent = 0.0f : 0);
    uint16_t answer = (uint16_t)(65536.0f*percent);
    return answer;
}

float32_t IBC::uint16_to_float32(uint16_t data, float data_max, float data_min) {
    float percent = ((float)data)/65536.0f;
    float range = data_max - data_min;
    float answer = data_min + range*percent;
    return answer;
}

