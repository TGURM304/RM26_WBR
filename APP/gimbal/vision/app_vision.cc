//
// Created by fish on 2024/12/18.
//

#include "app_vision.h"

#include <algorithm>
#include <cstring>

#include "app_ins.h"
#include "bsp_uart.h"
#include "alg_crc.h"
#include "bsp_def.h"
#include "bsp_time.h"

using namespace vision;

static RecvPacket rx_packet;

#define E_UART_VISION E_UART_10

// 新增：最近一次收到视觉数据的本地时间戳（ms）
static uint32_t last_update_time_ms = 0;

void uart_rx_callback(bsp_uart_e e, const uint8_t *s, size_t l) {
    if(l < sizeof rx_packet) return;
    std::copy_n(s, sizeof rx_packet, reinterpret_cast <uint8_t *> (&rx_packet));
    last_update_time_ms = bsp_time_get_ms();
}

void vision::init() {
    bsp_uart_set_callback(E_UART_VISION, uart_rx_callback);
}

RecvPacket *vision::recv() {
    return &rx_packet;
}

uint32_t vision::last_update_time() {
    return last_update_time_ms;
}

static auto ins = app_ins_data();
/**
 *
 * @param roll 云台roll
 * @param yaw 云台yaw
 * @param yaw_vel 云台yaw速度
 * @param pitch 云台pitch
 * @param pitch_vel 云台pitch速度
 * @param bullet_speed 弹速
 * @param bullet_count 弹数量
 * @param target_color 目标颜色,0是都瞄,'R' = 82 红色, 'B' == 66 蓝色
 */
void vision::send(float roll, float yaw, float yaw_vel, float pitch,
    float pitch_vel, float bullet_speed, uint16_t bullet_count, uint8_t target_color) {
    float q[4];
    SendPacket pkg = {};

    // 四元数计算
    float half_roll  = roll  * 0.5f;
    float half_pitch = pitch * 0.5f;
    float half_yaw   = yaw   * 0.5f;

    float cr = cosf(half_roll);
    float sr = sinf(half_roll);
    float cp = cosf(half_pitch);
    float sp = sinf(half_pitch);
    float cy = cosf(half_yaw);
    float sy = sinf(half_yaw);

    q[0] = cr * cp * cy + sr * sp * sy;  // w
    q[1] = sr * cp * cy - cr * sp * sy;  // x
    q[2] = cr * sp * cy + sr * cp * sy;  // y
    q[3] = cr * cp * sy - sr * sp * cy;  // z

    pkg.mode = 1;
    for (size_t i = 0; i < 4; i++)
        pkg.q[i] = q[i];
    pkg.yaw = yaw;
    pkg.yaw_vel = yaw_vel;
    pkg.pitch = pitch;
    pkg.pitch_vel = pitch_vel;
    pkg.bullet_speed = bullet_speed;
    pkg.bullet_count = bullet_count;
    pkg.color = target_color;
    // TODO: CRC 未测试
    CRC16::append(pkg);
    bsp_uart_send(E_UART_VISION, reinterpret_cast <uint8_t *> (&pkg), sizeof pkg);
}