//
// Created by fish on 2026/3/27.
//

#include "robomaster.h"
#include "bsp_time.h"
#include "alg_crc.h"

#include <cstring>

using namespace robomaster;

namespace robomaster::image {
    bsp_uart_e port;
    frame_header_t header;
    namespace rc {
        raw_frame_t raw;
        data_t rc_data_;
    }
    namespace custom {
        uint8_t* ptr; size_t size;
        uint32_t timestamp;
    };
    void callback(bsp_uart_e device, const uint8_t *data, size_t size);
}

const image::rc::data_t *image::rc::data() {
    return &rc_data_;
}

void image::init(bsp_uart_e uart) {
    static_assert(sizeof(rc::raw_frame_t) == 21);
    port = uart;
    bsp_uart_set_baudrate(uart, 921600);
    bsp_uart_set_callback(uart, callback);
}

void image::custom::bind(uint8_t* _ptr, size_t _size) {
    ptr = _ptr;
    size = _size;
}

uint32_t image::custom::get_timestamp() {
    return timestamp;
}

void image::callback(bsp_uart_e device, const uint8_t* data, size_t size) {
    size_t p = 0;
    while (p < size) {
        if (data[p] == 0xa9 and data[p+1] == 0x53 and p + sizeof(rc::raw_frame_t) <= size) {
            memcpy(&rc::raw, data+p, sizeof(rc::raw_frame_t));
            if (!CRC16::verify(rc::raw)) { p++; continue; }

            rc::rc_data_.l[0] = static_cast<int16_t>(static_cast<int16_t>(rc::raw.ch3) - 1024);
            rc::rc_data_.l[1] = static_cast<int16_t>(static_cast<int16_t>(rc::raw.ch2) - 1024);
            rc::rc_data_.r[0] = static_cast<int16_t>(static_cast<int16_t>(rc::raw.ch0) - 1024);
            rc::rc_data_.r[1] = static_cast<int16_t>(static_cast<int16_t>(rc::raw.ch1) - 1024);
            rc::rc_data_.dial = static_cast<int16_t>(static_cast<int16_t>(rc::raw.dial) - 1024);
            rc::rc_data_.sw = static_cast<int8_t>(rc::raw.sw - 1);

            rc::rc_data_.key_suspend = rc::raw.key_suspend;
            rc::rc_data_.key_l = rc::raw.key_l;
            rc::rc_data_.key_r = rc::raw.key_r;
            rc::rc_data_.key_shoot = rc::raw.key_shoot;

            rc::rc_data_.mouse_x = rc::raw.mouse_x;
            rc::rc_data_.mouse_y = rc::raw.mouse_y;
            rc::rc_data_.mouse_z = rc::raw.mouse_z;
            rc::rc_data_.mouse_l = rc::raw.mouse_l;
            rc::rc_data_.mouse_r = rc::raw.mouse_r;
            rc::rc_data_.mouse_m = rc::raw.mouse_m;
            rc::rc_data_.keyboard = rc::raw.keyboard;

            rc::rc_data_.timestamp = bsp_time_get_ms();

            p += sizeof(rc::raw_frame_t);
        } else if (data[p] == 0xa5) {
            memcpy(&header, data + p, sizeof(frame_header_t));
            if (header.sof != 0xa5 or !CRC8::verify(header)) { p ++; continue; }
            if (CRC16::calc(data + p, sizeof(frame_header_t) + 2 + header.data_length) !=
                *(uint16_t *)(data + p + sizeof(frame_header_t) + 2 + header.data_length))
            {
                p ++;
                continue;
            }

            uint16_t cmd_id = *(uint16_t *)(data + p + sizeof(frame_header_t));
            p += sizeof(frame_header_t) + 2;

            switch(cmd_id) {
            case 0x0302:
                if (custom::ptr != nullptr and custom::size == header.data_length)
                    memcpy(custom::ptr, data + p, custom::size), custom::timestamp = bsp_time_get_ms();
                break;
            default:
                break;
            }

            p += header.data_length + 2; // data + crc16
        } else {
            p ++;
        }
    }
}