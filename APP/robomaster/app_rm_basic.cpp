//
// Created by fish on 2026/2/3.
//

#include "robomaster.h"

#include <cstring>
#include <algorithm>

#include "bsp_time.h"
#include "alg_crc.h"
#include "sys_task.h"
#include "sys_queue.h"

using namespace robomaster;

static uint8_t ui_buf[256];
static uint8_t tx_buf[BSP_UART_DEVICE_COUNT][256];
OS::Queue<basic::ui::figure_pkg_t> ui_figure_queue(50);
OS::Queue<basic::ui::string_pkg_t> ui_string_queue(50);

void robomaster::transmit(bsp_uart_e device, uint16_t cmd_id, const uint8_t *data, uint16_t size) {
    if (size + sizeof(frame_header_t) + 4 > sizeof(tx_buf[device])) {
        return;
    }
    memset(tx_buf[device], 0, sizeof(tx_buf[device]));
    frame_header_t header = { .sof = 0xa5, .data_length = size, .seq = 0, .crc = 0 };
    CRC8::append(header);
    memcpy(tx_buf[device], &header, sizeof(header));
    memcpy(tx_buf[device] + sizeof(header), &cmd_id, 2);
    memcpy(tx_buf[device] + sizeof(header) + 2, data, size);
    auto crc = CRC16::calc(tx_buf[device], sizeof(header) + 2 + size, 0xffff);
    memcpy(tx_buf[device] + sizeof(header) + 2 + size, &crc, 2);
    bsp_uart_send_async(device, tx_buf[device], sizeof(header) + 2 + size + 2);
}

namespace robomaster::basic {
    bsp_uart_e port;
    data_t data_;
    frame_header_t header;
    void callback(bsp_uart_e device, const uint8_t *data, size_t size);
    namespace ui {
        [[noreturn]] void task(void *args);
    }
}

const basic::data_t* basic::data() {
    return &data_;
}

static OS::Task ui_task_handle;

void basic::init(bsp_uart_e uart) {
    port = uart;
    bsp_uart_set_baudrate(uart, 115200);
    bsp_uart_set_callback(uart, callback);
    ui_task_handle.Create(ui::task, static_cast<void *>(nullptr), "robomaster::ui", 512, OS::Task::Priority::HIGH);
}

void basic::callback(bsp_uart_e device, const uint8_t* data, size_t size) {
    if (size < sizeof(frame_header_t)) return;

    size_t p = 0;
    while (p + sizeof(frame_header_t) < size) {
        memcpy(&header, data + p, sizeof(frame_header_t));
        if (header.sof != 0xa5 or !CRC8::verify(header)) { p ++; continue; }
        if (CRC16::calc(data + p, sizeof(frame_header_t) + 2 + header.data_length, 0xffff) !=
            *(uint16_t *)(data + p + sizeof(frame_header_t) + 2 + header.data_length))
        {
            p ++;
            continue;
        }

        uint16_t cmd_id = *(uint16_t *)(data + p + sizeof(frame_header_t));
        p += sizeof(frame_header_t) + 2;

#define upd(x) if(header.data_length == sizeof(data_.x)) memcpy(&data_.x, data + p, sizeof(data_.x)), data_.timestamps.x = bsp_time_get_ms()
        switch(cmd_id) {
        case 0x0001: upd(game_status); break;
        case 0x0002: upd(game_result); break;
        case 0x0003: upd(game_robot_hp); break;
        case 0x0101: upd(event_data); break;
        case 0x0104: upd(referee_warning); break;
        case 0x0105: upd(dart_info); break;
        case 0x0201: upd(robot_status); break;
        case 0x0202: upd(power_heat_data); break;
        case 0x0203: upd(robot_pos); break;
        case 0x0204: upd(buff); break;
        case 0x0206: upd(hurt_data); break;
        case 0x0207: upd(shoot_data); break;
        case 0x0208: upd(projectile_allowance); break;
        case 0x0209: upd(rfid_status); break;
        case 0x020a: upd(dart_client_cmd); break;
        case 0x020b: upd(ground_robot_position); break;
        case 0x020c: upd(radar_mark_data); break;
        case 0x020d: upd(sentry_info); break;
        case 0x020e: upd(radar_info); break;
        default:
            break;
        }
#undef upd

        p += header.data_length + 2; // data + crc16
    }
}

void basic::ui::_add(const char* name, uint8_t figure_type, uint8_t layer, uint16_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t e) {
    figure_pkg_t pkg = {
        .operate_type = 1,
        .figure_type = figure_type,
        .layer = layer,
        .color = color,
        .details_a = a,
        .details_b = b,
        .width = width,
        .start_x = x,
        .start_y = y,
        .details_c = c,
        .details_d = d,
        .details_e = e
    };
    strcpy(pkg.figure_name, name);
    ui_figure_queue.send(pkg);
}

void basic::ui::_update(const char* name, uint8_t figure_type, uint8_t layer, uint16_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t e) {
    figure_pkg_t pkg = {
        .operate_type = 2,
        .figure_type = figure_type,
        .layer = layer,
        .color = color,
        .details_a = a,
        .details_b = b,
        .width = width,
        .start_x = x,
        .start_y = y,
        .details_c = c,
        .details_d = d,
        .details_e = e
    };
    strcpy(pkg.figure_name, name);
    ui_figure_queue.send(pkg);
}

void basic::ui::add_string(const char* name, uint8_t layer, uint16_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t font_size, const char* str) {
    string_pkg_t pkg = {
        .operate_type = 1,
        .figure_type = 7,
        .layer = layer,
        .color = color,
        .details_a = font_size,
        .details_b = std::min(sizeof(pkg.data), strlen(str)),
        .width = width,
        .start_x = x,
        .start_y = y
    };
    strcpy(pkg.figure_name, name);
    strcpy(pkg.data, str);
    ui_string_queue.send(pkg);
}

void basic::ui::update_string(const char* name, uint8_t layer, uint16_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t font_size, const char* str) {
    string_pkg_t pkg = {
        .operate_type = 2,
        .figure_type = 7,
        .layer = layer,
        .color = color,
        .details_a = font_size,
        .details_b = std::min(sizeof(pkg.data), strlen(str)),
        .width = width,
        .start_x = x,
        .start_y = y
    };
    strcpy(pkg.figure_name, name);
    strcpy(pkg.data, str);
    ui_string_queue.send(pkg);
}

void basic::ui::remove(const char* name, uint8_t layer) {
    figure_pkg_t pkg = {
        .operate_type = 3,
        .layer = layer
    };
    strcpy(pkg.figure_name, name);
    ui_figure_queue.send(pkg);
}

[[noreturn]] void basic::ui::task(void *args) {
    for (;;) {
        while (bsp_time_get_ms() - data()->timestamps.robot_status > 100 or (!ui_figure_queue.size() and !ui_string_queue.size())) {
            OS::Task::Sleep(1);
        }
        const uint16_t sender = data()->robot_status.robot_id, receiver = 0x0100 + sender;
        if (ui_string_queue.size()) {
            string_pkg_t pkg = {};
            interaction_header_t ui_header = {
                .data_cmd_id = 0x0110,
                .sender_id = sender,
                .receiver_id = receiver
            };
            memcpy(ui_buf, &ui_header, sizeof(ui_header));
            ui_string_queue.receive(pkg);
            memcpy(ui_buf + sizeof(ui_header), &pkg, sizeof(pkg));
            transmit(port, 0x0301, ui_buf, sizeof(ui_header) + sizeof(pkg));
        } else if (ui_figure_queue.size() >= 7) {
            figure_pkg_t pkg = {};
            interaction_header_t ui_header = {
                .data_cmd_id = 0x0104,
                .sender_id = sender,
                .receiver_id = receiver
            };
            memcpy(ui_buf, &ui_header, sizeof(ui_header));
            for (uint8_t i = 0; i < 7; i++) {
                ui_figure_queue.receive(pkg);
                memcpy(ui_buf + sizeof(ui_header) + sizeof(pkg) * i, &pkg, sizeof(pkg));
            }
            transmit(port, 0x0301, ui_buf, sizeof(ui_header) + sizeof(pkg) * 7);
        } else if (ui_figure_queue.size() >= 5) {
            figure_pkg_t pkg = {};
            interaction_header_t ui_header = {
                .data_cmd_id = 0x0103,
                .sender_id = sender,
                .receiver_id = receiver
            };
            memcpy(ui_buf, &ui_header, sizeof(ui_header));
            for (uint8_t i = 0; i < 5; i++) {
                ui_figure_queue.receive(pkg);
                memcpy(ui_buf + sizeof(ui_header) + sizeof(pkg) * i, &pkg, sizeof(pkg));
            }
            transmit(port, 0x0301, ui_buf, sizeof(ui_header) + sizeof(pkg) * 5);
        } else if (ui_figure_queue.size() >= 2) {
            figure_pkg_t pkg = {};
            interaction_header_t ui_header = {
                .data_cmd_id = 0x0102,
                .sender_id = sender,
                .receiver_id = receiver
            };
            memcpy(ui_buf, &ui_header, sizeof(ui_header));
            for (uint8_t i = 0; i < 2; i++) {
                ui_figure_queue.receive(pkg);
                memcpy(ui_buf + sizeof(ui_header) + sizeof(pkg) * i, &pkg, sizeof(pkg));
            }
            transmit(port, 0x0301, ui_buf, sizeof(ui_header) + sizeof(pkg) * 2);
        } else if (ui_figure_queue.size()) {
            figure_pkg_t pkg = {};
            interaction_header_t ui_header = {
                .data_cmd_id = 0x0101,
                .sender_id = sender,
                .receiver_id = receiver
            };
            memcpy(ui_buf, &ui_header, sizeof(ui_header));
            ui_figure_queue.receive(pkg);
            memcpy(ui_buf + sizeof(ui_header), &pkg, sizeof(pkg));
            transmit(port, 0x0301, ui_buf, sizeof(ui_header) + sizeof(pkg));
        }
        OS::Task::Sleep(35);
    }
}