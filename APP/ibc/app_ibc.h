//
// Created by 15082 on 2026/3/13.
//

#ifndef APP_IBC_H
#define APP_IBC_H

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
namespace IBC {
typedef struct {
    uint16_t vx,vy;
    uint16_t spin_rad;
    uint8_t switch_cmd;
}gimbal;
typedef struct {
    float vector_x,vector_y,vector_z;
    uint8_t mode;
}chassis;
int16_t float32_to_int16(float32_t data, float data_max, float data_min);
}


#endif //APP_IBC_H
