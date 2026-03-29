//
// Created by 15082 on 2026/3/28.
//

#ifndef APP_GIMBAL_COORDINATE_H
#define APP_GIMBAL_COORDINATE_H
#include "app_cmd.h"
#include "app_head_controller.h"
#include "app_second_order.h"
#include "app_shoot_controller.h"
#include "app_vision_core.h"

namespace Gimbal {
class gimbal_coordinate {
public:
    gimbal_coordinate() =default;
    gimbal_coordinate(Shoot* shoot, Head* head, vision_core* vision):
    shoot_(shoot), head_(head), vision_(vision),
    pitch_path(1000,30,1), yaw_path(1000,30,1) {}
    void init();
    void tick();
    void reset();
    void update_rc(const bsp_rc_data_t *rc);
    void update_mouse(Gimbal_cmd::mouse_pkg pkg_);
    void update_video();
    Gimbal_cmd::gimbal_ctrl ctrl_;

private:
    float raw_yaw, raw_pit;
    Shoot* shoot_;
    Head* head_;
    vision_core* vision_;
    PathReference::second_order pitch_path = {};
    PathReference::second_order yaw_path = {};
    float32_t target_pit_, target_yaw_;
};

}

#endif //APP_GIMBAL_COORDINATE_H
