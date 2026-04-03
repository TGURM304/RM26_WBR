//
// Created by 15082 on 2026/3/13.
//

#ifndef APP_IBC_H
#define APP_IBC_H

#include "app_behavior_define.h"

#include <arm_math_types.h>
#include <cstdint>
#include "app_msg.h"

#define CHASSIS_ID 0x36
#define GIMBAL_ID 0x37
/*
 * 对通讯协议进行规定
 * 发送通道：CAN3
 * 主板（云台）id：0x51
 * 从板（底盘）id：0x52
 * 云台发送内容：
 *      底盘vx，vy（4Byte）
 *      底盘小陀螺速度（2Byte）
 *      底盘切换命令（参考behavior）（1Byte）
 * 底盘反馈内容：
 *      底盘三个矢量Vector_x, Vector_y, Vector_z（6Byte）
 *      底盘当前工作模式（1Byte）
 */


#define ZH_GRY_MAX (8.0f)
#define ZH_GRY_MIN (-8.0f)
#define ZH_V_MAX (4.0f)
#define ZH_V_MIN (-4.0f)
#define ZH_HEIGHT_MAX (0.4f)
#define ZH_HEIGHT_MIN (0.15f)

namespace IBC {
typedef struct {
    uint16_t vx,vy,target_yaw,height;//8Byte
    uint16_t gry;
    uint8_t spin_flag;
    bool reset;
    Coordinate::mode_switch_cmd switch_cmd;
} __attribute__((packed)) ibc_gimbal;
typedef struct {
    uint16_t vector_x,vector_y,vector_z;//6Byte
    uint16_t body_phi;//2Byte
    Coordinate::mode_state chassis_cmd_;
} __attribute__((packed)) ibc_chassis;
typedef struct {
    float vector_x,vector_y,vector_z;
    float body_phi;
    Coordinate::mode_state chassis_cmd_;
}ibc_chassis_data;
typedef struct {
    float vx,vy,target_yaw,height;
    float gry;
    Coordinate::mode_switch_cmd switch_cmd;
    bool reset;
}ibc_gimbal_data;
uint16_t float32_to_uint16(float32_t data, float data_max, float data_min);
float32_t uint16_to_float32(uint16_t data, float data_max, float data_min);
}


#endif //APP_IBC_H
