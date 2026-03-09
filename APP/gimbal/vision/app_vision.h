//
// Created by fish on 2024/12/18.
//

#pragma once

#include <cstdint>

namespace vision {
    struct SendPacket {
        uint8_t head[2] = {'T', 'G'};
        uint8_t mode;           // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
        float q[4];             // wxyz顺序
        float yaw;              // 实际偏航角(弧度制)
        float yaw_vel;          // 实际偏航角速度
        float pitch;            // 实际俯仰角(弧度制)
        float pitch_vel;        // 实际俯仰角速度
        float bullet_speed;     // 弹速
        uint16_t bullet_count;  // 累计发弹数
        uint16_t crc16;
    } __attribute__((packed));
    static_assert(sizeof(SendPacket) <= 64);

    struct RecvPacket {
        uint8_t head[2] = {'T', 'G'};
        uint8_t mode;           // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
        float yaw;              // 目标偏航角(弧度制)
        float yaw_vel;          // 目标偏航角速度
        float yaw_acc;          // 目标偏航角加速度
        float pitch;            // 目标俯仰角(弧度制)
        float pitch_vel;        // 目标俯仰角速度
        float pitch_acc;        // 目标俯仰角加速度
        uint16_t crc16;
    } __attribute__((packed));
    static_assert(sizeof(RecvPacket) <= 64);

    void init();
    RecvPacket *recv();
    void send(float roll, float yaw, float yaw_vel, float pitch, float pitch_vel, float bullet_speed, uint16_t bullet_count);
    uint32_t last_update_time();
}