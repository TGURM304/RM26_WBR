//
// Created by fish on 2026/2/3.
//

#pragma once

#include "app_rm_def.h"
#include "bsp_uart.h"

namespace robomaster {
    void transmit(bsp_uart_e device, uint16_t cmd_id, const uint8_t *data, uint16_t size);
    // 常规链路
    namespace basic {
        struct data_t {
            // 0x0001 比赛状态数据, 固定以 1Hz 频率发送, 服务器->全体机器人
            game_status_t game_status;
            // 0x0002 比赛结果数据, 比赛结束触发发送, 服务器->全体机器人
            game_result_t game_result;
            // 0x0003 机器人血量数据, 固定以 3Hz 频率发送, 服务器->全体机器人
            game_robot_hp_t game_robot_hp;
            // 0x0101 场地事件数据, 固定以 1Hz 频率发送, 服务器->己方全体机器人
            event_data_t event_data;
            // 0x0104 裁判警告数据, 己方判罚/判负时触发发送, 其余时间以 1Hz 频率发送, 服务器->被判罚方全体机器人
            referee_warning_t referee_warning;
            // 0x0105 飞镖发射相关数据, 固定以 1Hz频率发送, 服务器->己方全体机器人
            dart_info_t dart_info;
            // 0x0201 机器人性能体系数据, 固定以 10Hz 频率发送, 主控模块->对应机器人
            robot_status_t robot_status;
            // 0x0202 实时底盘缓冲能量和射击热量数据, 固定以 10Hz 频率发送, 主控模块->对应机器人
            power_heat_data_t power_heat_data;
            // 0x0203 机器人位置数据, 固定以 1Hz 频率发送, 主控模块->对应机器人
            robot_pos_t robot_pos;
            // 0x0204 机器人增益和底盘能量数据, 固定以 3Hz 频率发送, 服务器->对应机器人
            buff_t buff;
            // 0x0206 伤害状态数据, 伤害发生后发送, 主控模块->对应机器人
            hurt_data_t hurt_data;
            // 0x0207 实时射击数据, 弹丸发射后发送, 主控模块->对应机器人
            shoot_data_t shoot_data;
            // 0x0208 允许发弹量, 固定以 10Hz 频率发送, 服务器->己方英雄、步兵、哨兵、空中机器人
            projectile_allowance_t projectile_allowance;
            // 0x0209 机器人 RFID 模块状态, 固定以 3Hz 频率发送, 服务器->己方装有 RFID 模块的机器人
            rfid_status_t rfid_status;
            // 0x020a 飞镖选手端指令数据, 固定以 3Hz 频率发送, 服务器->己方飞镖机器人
            dart_client_cmd_t dart_client_cmd;
            // 0x020b 地面机器人位置数据, 固定以 1Hz 频率发送, 服务器->己方哨兵机器人
            ground_robot_position_t ground_robot_position;
            // 0x020c 雷达标记进度数据, 固定以 1Hz 频率发送, 服务器->己方雷达机器人
            radar_mark_data_t radar_mark_data;
            // 0x020d 哨兵自主决策信息同步, 固定以 1Hz 频率发送, 服务器->己方哨兵机器人
            sentry_info_t sentry_info;
            // 0x020e 雷达自主决策信息同步, 固定以 1Hz 频率发送, 服务器->己方雷达机器人
            radar_info_t radar_info;
            struct {
                uint32_t game_status;
                uint32_t game_result;
                uint32_t game_robot_hp;
                uint32_t event_data;
                uint32_t referee_warning;
                uint32_t dart_info;
                uint32_t robot_status;
                uint32_t power_heat_data;
                uint32_t robot_pos;
                uint32_t buff;
                uint32_t hurt_data;
                uint32_t shoot_data;
                uint32_t projectile_allowance;
                uint32_t rfid_status;
                uint32_t dart_client_cmd;
                uint32_t ground_robot_position;
                uint32_t radar_mark_data;
                uint32_t sentry_info;
                uint32_t radar_info;
            } timestamps;
        };

        void init(bsp_uart_e uart);
        const data_t *data();

        namespace ui {
            enum figure_type_e {
                UI_FIGURE_LINE = 0,            // 直线
                UI_FIGURE_RECTANGLE = 1,       // 矩形
                UI_FIGURE_CIRCLE = 2,          // 正圆
                UI_FIGURE_OVAL = 3,            // 椭圆
                UI_FIGURE_ARC = 4,             // 圆弧
                UI_FIGURE_FLOAT = 5,           // 浮点数
                UI_FIGURE_INT = 6,             // 整型数
                UI_FIGURE_STRING = 7           // 字符
            };
            enum color_e {
                UI_COLOR_SELF,
                UI_COLOR_YELLOW,
                UI_COLOR_GREEN,
                UI_COLOR_ORANGE,
                UI_COLOR_PURPLE,
                UI_COLOR_PINK,
                UI_COLOR_CYAN,
                UI_COLOR_BLACK,
                UI_COLOR_WHITE,
            };
            void _add(const char *name, uint8_t figure_type, uint8_t layer, uint16_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t e);
            void _update(const char *name, uint8_t figure_type, uint8_t layer, uint16_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t e);
            void add_string(const char *name, uint8_t layer, uint16_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t font_size, const char *str);
            void update_string(const char *name, uint8_t layer, uint16_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t font_size, const char *str);
            void remove(const char *name, uint8_t layer);

            inline void add_line(const char *name, uint8_t layer, uint16_t color, uint32_t width, uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2) {
                _add(name, UI_FIGURE_LINE, layer, color, width, x1, y1, 0, 0, 0, x2, y2);
            }

            inline void add_rectangle(const char *name, uint8_t layer, uint16_t color, uint32_t width, uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2) {
                _add(name, UI_FIGURE_RECTANGLE, layer, color, width, x1, y1, 0, 0, 0, x2, y2);
            }

            inline void add_circle(const char *name, uint8_t layer, uint16_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t r) {
                _add(name, UI_FIGURE_CIRCLE, layer, color, width, x, y, 0, 0, r, 0, 0);
            }

            inline void add_oval(const char *name, uint8_t layer, uint16_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t rx, uint32_t ry) {
                _add(name, UI_FIGURE_OVAL, layer, color, width, x, y, 0, 0, 0, rx, ry);
            }

            inline void add_arc(const char *name, uint8_t layer, uint16_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t al, uint32_t ar, uint32_t rx, uint32_t ry) {
                _add(name, UI_FIGURE_ARC, layer, color, width, x, y, al, ar, 0, rx, ry);
            }

            inline void add_float(const char *name, uint8_t layer, uint16_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t font_size, float val) {
                union { int32_t n; struct { uint32_t c : 10, d : 11, e : 11; } __attribute__((packed)) s; } u = {
                    .n = static_cast <int32_t> (val * 1000)
                };
                _add(name, UI_FIGURE_FLOAT, layer, color, width, x, y, font_size, 0, u.s.c, u.s.d, u.s.e);
            }

            inline void add_int(const char *name, uint8_t layer, uint16_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t font_size, int32_t val) {
                union { int32_t n; struct { uint32_t c : 10, d : 11, e : 11; } __attribute__((packed)) s; } u = {
                    .n = val
                };
                _add(name, UI_FIGURE_INT, layer, color, width, x, y, font_size, 0, u.s.c, u.s.d, u.s.e);
            }

            inline void update_line(const char *name, uint8_t layer, uint16_t color, uint32_t width, uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2) {
                _update(name, UI_FIGURE_LINE, layer, color, width, x1, y1, 0, 0, 0, x2, y2);
            }

            inline void update_rectangle(const char *name, uint8_t layer, uint16_t color, uint32_t width, uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2) {
                _update(name, UI_FIGURE_RECTANGLE, layer, color, width, x1, y1, 0, 0, 0, x2, y2);
            }

            inline void update_circle(const char *name, uint8_t layer, uint16_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t r) {
                _update(name, UI_FIGURE_CIRCLE, layer, color, width, x, y, 0, 0, r, 0, 0);
            }

            inline void update_oval(const char *name, uint8_t layer, uint16_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t rx, uint32_t ry) {
                _update(name, UI_FIGURE_OVAL, layer, color, width, x, y, 0, 0, 0, rx, ry);
            }

            inline void update_arc(const char *name, uint8_t layer, uint16_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t al, uint32_t ar, uint32_t rx, uint32_t ry) {
                _update(name, UI_FIGURE_ARC, layer, color, width, x, y, al, ar, 0, rx, ry);
            }

            inline void update_float(const char *name, uint8_t layer, uint16_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t font_size, float val) {
                union { int32_t n; struct { uint32_t c : 10, d : 11, e : 11; } __attribute__((packed)) s; } u = {
                    .n = static_cast <int32_t> (val * 1000)
                };
                _update(name, UI_FIGURE_FLOAT, layer, color, width, x, y, font_size, 0, u.s.c, u.s.d, u.s.e);
            }

            inline void update_int(const char *name, uint8_t layer, uint16_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t font_size, int32_t val) {
                union { int32_t n; struct { uint32_t c : 10, d : 11, e : 11; } __attribute__((packed)) s; } u = {
                    .n = val
                };
                _update(name, UI_FIGURE_INT, layer, color, width, x, y, font_size, 0, u.s.c, u.s.d, u.s.e);
            }
        }
    }
    // 图传链路
    namespace image {
        // 新图传接收端遥控数据
        namespace rc {
            struct data_t {
                int16_t l[2], r[2], dial, mouse_x, mouse_y, mouse_z;
                int8_t sw;
                bool key_suspend, key_l, key_r, key_shoot, mouse_l, mouse_r, mouse_m;
                uint16_t keyboard;
                uint32_t timestamp;
            };
            const data_t *data();
        }
        namespace custom {
            void bind(uint8_t *ptr, size_t size);
            template <typename T>
            void bind(T &data) { bind(reinterpret_cast<uint8_t *>(&data), sizeof(T)); }
            uint32_t get_timestamp();
        }
        void init(bsp_uart_e uart);
    }
    // 雷达无线链路
    namespace radar {
        // void init(bsp_uart_e uart);
    }
}