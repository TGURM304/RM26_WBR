//
// Created by 15082 on 2026/3/13.
//

#include "app_ibc.h"

int16_t IBC::float32_to_int16(float32_t data, float data_max, float data_min) {
    float data_range = data_max - data_min;
    float percent = (data - data_min)/data_range;
    int16_t answer = 65535*percent;
    return answer;
}

