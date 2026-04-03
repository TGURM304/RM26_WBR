//
// Created by 15082 on 2026/3/30.
//

#include "app_keyboard.h"

#include "app_coordinate.h"

void Gimbal::keyboard::update() {
    if (rc_data_ == nullptr) {
        return;
    }

    // ===== 可调参数 =====
    constexpr float kMoveSpeedNormal = 1.50f;
    constexpr float kMoveSpeedFast   = 3.0f;

    constexpr float kSpinSpeedNormal = 4.0f;
    constexpr float kSpinSpeedFast   = 6.0f;

    constexpr float kHeightStep = 0.0001f;

    constexpr float mouse_K = 1/30.0f;

    pkg_.mouse_x = -rc_data_->mouse_x * mouse_K;
    pkg_.mouse_y = -rc_data_->mouse_y * mouse_K;
    // 上一帧键盘状态
    bsp_rc_keyboard_u last_keyboard{};
    last_keyboard.raw = keyboard_raw.raw;
    // 读取当前键盘原始值
    keyboard_raw.raw = rc_data_->keyboard;
    const auto &key = keyboard_raw.key;
    const bool shift = key.shift;

    const auto &last_key = last_keyboard.key;

    // =========================
    // 1. follow_flag：按 V 切换
    // =========================
    if (key.v && !last_key.v) {
        pkg_.follow_flag = !pkg_.follow_flag;
    }

    // =========================
    // 2. fric_flag：按 F 切换
    // =========================
    if (key.f && !last_key.f) {
        pkg_.fric_flag = !pkg_.fric_flag;
    }

    // =========================
    // 3. shoot_fre：按 shift+R 三档循环切换 0/1/2
    // =========================
    if ((key.shift && key.r) && !last_key.r) {
        pkg_.shoot_fre = (pkg_.shoot_fre + 1) % 3;
    }

    // =========================
    // 4. auto_aim_flag：鼠标右键按下时打开
    // 如果你想“按住才开”，就这样写
    // =========================
    pkg_.auto_aim_flag = rc_data_->mouse_r;

    // =========================
    // 5. spin：Q/E 控制旋转，shift 加速
    // 约定：Q 为正方向，E 为负方向
    // 同时按下则相互抵消
    // =========================
    {
        const float spin_speed = shift ? kSpinSpeedFast : kSpinSpeedNormal;
        pkg_.spin = 0.0f;

        if (key.q && !key.e) {
            pkg_.spin = spin_speed;
        } else if (key.e && !key.q) {
            pkg_.spin = -spin_speed;
        }
    }

    // =========================
    // 6. vx / vy：WASD 合成平移速度，shift 加速
    // 这里约定：
    // W/S -> vx 正/负
    // A/D -> vy 正/负
    // =========================
    {
        const float move_speed = shift ? kMoveSpeedFast : kMoveSpeedNormal;
        pkg_.vx = 0.0f;
        pkg_.vy = 0.0f;

        float temp_x = 0, temp_y = 0;
        if (key.w && !key.s) {
            temp_x = 1;
        } else if (key.s && !key.w) {
            temp_x = -1;
        }

        if (key.a && !key.d) {
            temp_y = 1;
        } else if (key.d && !key.a) {
            temp_y = -1;
        }
        float deg = atan2f(temp_y, temp_x);
        if(temp_y == 0 && temp_x == 0) {
            deg = 0;//防止 atan2f(0,0) 出 NaN
            pkg_.vx = 0;
            pkg_.vy = 0;
        }
        else {
            pkg_.vx = cosf(deg) * move_speed;
            pkg_.vy = sinf(deg) * move_speed;
        }

    }

    // =========================
    // 7. target_height：shift+G 抬高，shift+B 降低
    // 这里写成“每次 update 累加/累减一步”
    // =========================
    if ((key.g && key.shift) && !key.b) {
        pkg_.target_height += kHeightStep;
    } else if ((key.b && key.shift) && !key.g) {
        pkg_.target_height -= kHeightStep;
    }
    pkg_.target_height > HEIGHT_MAX?
        pkg_.target_height = HEIGHT_MAX:
        (pkg_.target_height<HEIGHT_MIN?pkg_.target_height=HEIGHT_MIN:0);

    // =========================
    // 8. player_fire：玩家开火命令
    // 这里按常见逻辑用鼠标左键
    // 如果你想把 key_shoot 也算进去，可以保留下面这种写法
    // =========================
    pkg_.player_fire = rc_data_->mouse_l;

    // =========================
    // 9. 按下R直接休息
    // 不能与shift+R冲突
    // =========================
    if ((key.r) && !key.shift) {
        pkg_.switch_rest = true;
    }
    else {
        pkg_.switch_rest = false;
    }

    // =========================
    // 10. 按下V+shift重新enable一次
    // 不能与shift+G冲突
    // =========================
    if(key.v && !key.shift && !last_key.v)
        pkg_.reset_flag = true;
    else
        pkg_.reset_flag = false;

    // =========================
    // 11. 按下V+shift重新enable一次
    // 不能与shift+G冲突
    // =========================
    if(key.g && !key.shift && !last_key.g)
        pkg_.gimbal_flag = true;
    else
        pkg_.gimbal_flag = false;

    //底盘运行模式切换
    //土狗模式，腿部校准模式，LQR模式三选一，按Z/X/C切换，不能同时按，不能与休息冲突
    if(pkg_.switch_rest == false) {
        pkg_.switch_reset = false;
        pkg_.switch_dog = false;
        pkg_.switch_put = false;
        pkg_.switch_lqr = false;
        if(key.z && !key.x && !key.c && key.shift) {
            pkg_.switch_reset = true;
        }
        else if(key.z && !key.x && !key.c && !key.shift) {
            pkg_.switch_put = true;
        }
        else if(!key.z && key.x && !key.c && !key.shift) {
            pkg_.switch_dog = true;
        }
        else if(!key.z && !key.x && key.c && !key.shift) {
            pkg_.switch_lqr = true;
        }
    }
}
