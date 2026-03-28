//
// Created by 15082 on 2026/3/28.
//

#ifndef APP_CMD_H
#define APP_CMD_H

#include "app_behavior_define.h"
#include "app_shoot_controller.h"
#include "bsp_rc.h"
/**
 * 分为三个部分：
 * 控制命令更新模块
 * 机器人控制flag
 * 机器人上下板通讯
 */


#define USE_RC
// #define USE_MOUSE

namespace Gimbal_cmd {

typedef enum {
    E_NOT_FOLLOW,
    E_SPIN,
    E_FOLLOW
}body_state_;

/**
 * 控制命令更新模块：
 * 运动：
 * vx，vy，非小陀螺模式下机身delta_yaw
 * 头：
 * 是否发射
 * delta_pit，delta_yaw
 *
 */
typedef struct {
    float vx, vy, delta_body_yaw;
    bool shoot_flag;
    float delta_pit,delta_head_yaw;
}gimbal_cmd_pkg;

typedef struct {
    body_state_ car_ctrl_flag_;//底盘控制模式flag
    bool gimbal_ctrl_;//是否启用普通控制
    bool shoot_ctrl_;//是否启用手动发射
    bool auto_aim_gimbal_;//是否启用自瞄云台
    bool auto_aim_shoot_;//是否启用自瞄发弹
    Coordinate::mode_state chassis_sate_;//期望的state
    Coordinate::mode_switch_cmd switch_cmd_;//切换的命令
    Gimbal::trigger_mode_e trigger_mode_;//拨弹盘速度
    Gimbal::fric_mode_e fric_mode_;//发射机构速度
}gimbal_flag_pkg;


//底盘不是LQR就是土狗模式，只有自旋和前后两个自由度，没有侧移，所有的速度和目标值分配在Gimbal中完成
typedef struct {
    Coordinate::mode_switch_cmd switch_cmd_;
    Coordinate::mode_state chassis_cmd_;
    float dot_S, delta_yaw_;
}ibc_gimbal_send_pkg;
typedef struct {
    float vector_x,vector_y,vector_z;
    Coordinate::mode_state chassis_cmd_;
}ibc_chassis_send_pkg;
//最终的汇总的class,目前可以通过修改define来实现RC与裁判系统

typedef struct {
    //基础移动与开火
    float mouse_delta_x, mouse_delta_y;
    bool mouse_left, mouse_right;
    bool key_w, key_a, key_s, key_d;
    //特殊功能,为了防止误触，需要长按0.5S
    bool key_c;//超电
    bool key_v;//摆正机体位置

    bool key_q;//放置腿位姿
    bool key_e;//lqr启动
    bool key_r;//切换至土狗

    bool key_f;//切换开火模式
}mouse_pkg;
class gimbal_ctrl{
public:
    gimbal_ctrl():ibc_chassis_(E_CAN3,CHASSIS_ID) {
    }
    void init();
    void tick();
    void reset();
    void mouse_update(mouse_pkg pkg_);
    void rc_update(const bsp_rc_data_t *rc);
    ibc_chassis_send_pkg* get_chassis_pkg();
    ibc_gimbal_send_pkg* get_gimbal_pkg();
    gimbal_cmd_pkg* get_cmd() {return &cmd_;}
    gimbal_flag_pkg* get_flag() {return &flag_;};
private:
    ibc_gimbal_send_pkg gimbal_pkg_ = {};
    ibc_chassis_send_pkg chassis_pkg_ = {};
    app_msg_can_receiver<IBC::ibc_chassis> ibc_chassis_;

    gimbal_cmd_pkg cmd_{};
    gimbal_flag_pkg flag_{};
};
}




#endif //APP_CMD_H
