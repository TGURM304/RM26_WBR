//
// Created by 15082 on 2026/3/30.
//

#ifndef APP_KEYBOARD_H
#define APP_KEYBOARD_H
#include "bsp_rc.h"
#include "robomaster.h"

namespace Gimbal {
typedef struct {
    bool follow_flag;
    bool auto_aim_flag;
    bool fric_flag;
    bool player_fire;
    bool reset_flag;
    bool switch_dog, switch_put, switch_reset, switch_lqr, switch_rest;
    uint8_t shoot_fre;//0,1,2分别对应不同射速
    float vx,vy;
    float target_height;
    float spin;
    float mouse_x, mouse_y;
    bool gimbal_flag;
}keyboard_cmd_pkg;
class keyboard {
public:
    keyboard() = default;
    keyboard(const robomaster::image::rc::data_t *rc_data) : rc_data_(rc_data) {}
    void update();
    bsp_rc_keyboard_u* get_raw() {return &keyboard_raw;}
    keyboard_cmd_pkg get_pkg() {return pkg_;}
    float heart_beat_time();//确认是否活着
private:
    const robomaster::image::rc::data_t *rc_data_;
    bsp_rc_keyboard_u keyboard_raw = {};
    keyboard_cmd_pkg pkg_ = {};
    float last_heart_beat_time_ = 0.0f;
};

}



#endif //APP_KEYBOARD_H
