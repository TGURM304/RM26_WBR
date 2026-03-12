//
// Created by 15082 on 2026/3/10.
//

#include "app_vision_core.h"

#include "app_vision.h"

void Gimbal::vision_core::init() {
    //初始化自瞄回调函数
    vision::init();
}

void Gimbal::vision_core::tick() {
    current_cnt++;
    if(current_cnt < target_cnt) {
        return;
    }
    current_cnt = 0;
    auto pkg= snap_ptr_->get_snap_pkg();

    vision::send(pkg.ins_rol, pkg.ins_yaw, pkg.ins_yaw_dot,
        pkg.ins_pit, pkg.ins_pit_dot,
        3e8    , 1);
}

Gimbal::vision_pkg Gimbal::vision_core::get_target() {
    pkg_.fire_ctrl_cmd = vd->mode;
    pkg_.target_yaw = vd->yaw;
    pkg_.target_pitch = vd->pitch;
    return pkg_;
}