//
// Created by 15082 on 2026/3/14.
//

#ifndef APP_BEHAVIOR_DEFINE_H
#define APP_BEHAVIOR_DEFINE_H

#include "app_LQR.h"
#include "app_dog_ctrl.h"
#include "app_leg_ctrl.h"

namespace Coordinate {
typedef enum {
    E_NOT_READY,
    E_READY,
    E_FIRST,
    E_LEG_SHOU,
    E_STAND_READY,
    E_STAND
}upstairs_stage;
typedef enum {
    E_ANY,
    E_WAITING,
    E_PUT_BODY,
    E_PUT_LEG,
    E_DOG,
    E_CHAIR,
    E_LQR,
    E_UPSTAIRS,
    E_FALL_PROTECT,
    E_OFF_GROUND,
    E_GET_GROUND_SMOOTH,
    E_MODE_CNT
}mode_state;//转换模式
typedef enum {
    CMD_EXECUTING,
    CMD_START,
    CMD_DOG_START,
    CMD_DOG_END,
    CMD_CHAIR_START,
    CMD_STAIR_FINISH,
    CMD_STAIR_START,
    CMD_NORMAL_LQR,
    CMD_OFF_GROUND,
    CMD_GET_GROUND,
    CMD_SMOOTH_FINISH,
    CMD_FALL_DOWN,
    CMD_REBOOT,
    CMD_EMERGENCY,
    CMD_WAITING,
    CMD_LEG_START,
    CMD_CNT
}mode_switch_cmd;//转换条件或者说命令
typedef struct {
    float tor_j1, tor_j2, tor_j3, tor_j4;
    float dynamic_left, dynamic_right;
}out_tor;//输出扭矩
typedef struct {
    mode_state current_state;
    mode_switch_cmd switch_cmd;
    mode_state next_state;
} move_define; //状态转换指针
typedef struct {
    VMC::app_vmc *vmc;
    VMC::ctrl_pkg left_vmc_pkg, right_vmc_pkg;
    LegController::app_leg_ctrl *leg_ctrl;
    LQR::LQR_controller *lqr_controller;
    LegController::wheel_speed_controller *dog_ctrl;
} robot_controller_struct;//控制器结构体
typedef struct {
    Motor_Pkg::Joint *j1;
    Motor_Pkg::Joint *j2;
    Motor_Pkg::Joint *j3;
    Motor_Pkg::Joint *j4;
    Motor_Pkg::Dynamic *right;
    Motor_Pkg::Dynamic *left;
} component;//电机组件指针
typedef struct {
    mode_switch_cmd extern_cmd_, inner_cmd_;
    mode_state current_state_,last_state;
    uint16_t reduce_cnt;
    float delta_S;
    float delta_yaw;
    float height_record;
} mode_state_struct;
typedef struct {
    float32_t body_height, speed, gry;
    float32_t ver_x, ver_y;//相对云台的，云台正方向为x轴正方向
    bool spin_flag;
} ctrl_struct;
typedef struct {
    upstairs_stage stage_;
    float exe_cnt_;
    bool upstairs_flag;
    float left_deg_target;
    float right_deg_target;
    float left_len_target;
    float right_len_target;
    Relay::relay_lqr start_state;
    Relay::relay_leg left_leg;
    Relay::relay_leg right_leg;
} upstairs_struct;
static const int16_t size_map = 14+1;
inline move_define move_map[size_map] = {
    /* 等待状态 */
    {E_WAITING,            CMD_START,          E_PUT_BODY},
    /* 机体归正 */
    {E_PUT_BODY,           CMD_LEG_START,    E_PUT_LEG},
    /* 腿部归正 */
    {E_PUT_LEG,           CMD_DOG_START,    E_DOG},
    {E_PUT_LEG,           CMD_CHAIR_START,    E_CHAIR},
    /* 土狗模式 */
    {E_DOG,                CMD_DOG_END,        E_PUT_BODY},
    /* 小板凳起立 */
    {E_CHAIR,              CMD_NORMAL_LQR,     E_LQR},
    {E_CHAIR,              CMD_STAIR_START,    E_UPSTAIRS},
    /* 上台阶模式 */
    {E_UPSTAIRS,           CMD_STAIR_FINISH,   E_PUT_BODY},
    /* 正常LQR控制 */
    {E_LQR,                CMD_STAIR_START,    E_UPSTAIRS},
    {E_LQR,                CMD_OFF_GROUND,     E_OFF_GROUND},
    {E_LQR,                CMD_FALL_DOWN,      E_FALL_PROTECT},
    /* 离地控制 */
    {E_OFF_GROUND,         CMD_GET_GROUND,     E_GET_GROUND_SMOOTH},
    /* 落地缓冲 */
    {E_GET_GROUND_SMOOTH,  CMD_SMOOTH_FINISH,  E_LQR},
    /* 倒地保护 */
    {E_FALL_PROTECT,       CMD_REBOOT,         E_WAITING},
    {E_ANY,                CMD_EMERGENCY,      E_FALL_PROTECT}
};
}



#endif //APP_BEHAVIOR_DEFINE_H
