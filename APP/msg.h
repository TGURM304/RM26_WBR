//
// Created by fish on 2026/3/25.
//

#pragma once

#include <cstring>

#include "alg_crc.h"
#include "sys_task.h"
#include "sys_queue.h"
#include "bsp_can.h"
#include "bsp_time.h"

namespace msg {
    template <typename T, bsp_can_e _port, uint32_t _id, int queue_size = 64>
    class can_sender {
    public:
        can_sender(uint8_t pkgs_per_ms = 1)
            : _pkgs_per_ms(pkgs_per_ms), _pkg_queue(queue_size) {}
        T *operator()() { return &data; }
        void init() {
            OS::Task().Create([](void *args) {
                auto self = static_cast<can_sender *>(args);
                for (;;) {
                    uint8_t cur_pkgs = 0;
                    while (self->_pkg_queue.size() > 0 and cur_pkgs < self->_pkgs_per_ms) {
                        can_pkg_t pkg; self->_pkg_queue.receive(pkg);
                        bsp_can_send(_port, _id, pkg.data);
                        cur_pkgs ++;
                    }
                    OS::Task::Sleep(1);
                }
            }, static_cast<void *>(this), "can_sender", 128, OS::Task::Priority::REALTIME);
        }
        void send() {
            constexpr uint8_t pkg_num = (sizeof(T) + 4 + 8 - 1) / 8;
            static_assert(pkg_num <= queue_size);

            can_pkg_t pkg;
            uint8_t tmp[pkg_num * 8];
            memset(tmp, 0, sizeof(tmp));

            // 0xa5 [length] [data] [crc16]
            tmp[0] = 0xa5;
            tmp[1] = sizeof(T);
            memcpy(tmp + 2, &data, sizeof(T));
            *reinterpret_cast<uint16_t*>(tmp + 2 + sizeof(T)) =  CRC16::calc(tmp, sizeof(T) + 2, 0xffff);

            for (uint8_t i = 0; i < pkg_num; i++) {
                memcpy(pkg.data, tmp + i * 8, 8);
                _pkg_queue.send(pkg);
            }
        }
        struct can_pkg_t {
            uint8_t data[8];
        } __attribute__((packed));
        uint8_t _pkgs_per_ms;
        OS::Queue <can_pkg_t> _pkg_queue;
    private:
        T data;
    };

    template <typename T, bsp_can_e _port, uint32_t _id>
    class can_receiver {
    public:
        can_receiver() = default;
        void init() {
            self = this;
            bsp_can_set_callback(_port, _id, &can_receiver::callback);
        }
        T *operator()() { return &_data; }
        uint32_t timestamp = 0;
    private:
        T _data;
        uint8_t buf[((sizeof(T) + 4 + 8 - 1) / 8) * 8] = {}, ptr = 0;
        static inline can_receiver *self = nullptr;

        static void callback(bsp_can_msg_t *msg) { self->handle(msg->data, msg->header.DataLength); }
        void handle(const uint8_t *data, size_t len) {
            if (ptr == 0) {
                if (data[0] != 0xa5 or data[1] != sizeof(T)) return;
                memcpy(buf, data, len); ptr += len;
            } else {
                memcpy(buf + ptr, data, len); ptr += len;
            }
            if (ptr >= sizeof(T) + 4) {
                if (CRC16::calc(buf, sizeof(T) + 2, 0xffff) != *reinterpret_cast<uint16_t*>(buf + sizeof(T) + 2)) {
                    ptr = 0;
                    return;
                }
                memcpy(&_data, buf + 2, sizeof(T));
                timestamp = bsp_time_get_ms();
                ptr = 0;
            }
        }
    };
}
