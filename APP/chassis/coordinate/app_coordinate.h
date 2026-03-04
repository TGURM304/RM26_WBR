//
// Created by 15082 on 2026/2/12.
//

#ifndef APP_COORDINATE_H
#define APP_COORDINATE_H
#include "app_LQR.h"
#include "app_leg_ctrl.h"
#include "app_snap.h"
#include "app_vmc.h"
#include "bsp_rc.h"
#include "app_biquad_filter.h"

#include <vector>

#define REDUCE_EDGE_DOWN   300
#define REDUCE_EDGE_UP    500
#define DELTA_S_EDGE    0.40f
#define DELTA_VER_EDGE  2.5f

#define FILTER_ORDER 2
#define FILTER_FC_HIGH 100.0f
#define FILTER_FC_LOW 10.0f
#define FILTER_FS 1000.0f

namespace Coordinate {
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
    float height_record;
} mode_state_struct;
typedef struct {
    float32_t body_height, speed, gryo;
    bool spin_flag;
}ctrl_struct;
    class app_coordinate {
    public:
        app_coordinate();
        app_coordinate(snap* robot_snap, robot_controller_struct controller, component motor_component)
        :robot_snap_ptr_(robot_snap), controller_(controller), motor_component_(motor_component)
         {

        };
        void tick();
        void test_function(const bsp_rc_data_t *rc);
        void reset();
        void inner_cmd_update();
    private:

        void exe_any                (snap *robot_snap,mode_state_struct state,ctrl_struct ctrl);
        void exe_waiting            (snap *robot_snap,mode_state_struct state,ctrl_struct ctrl);
        void exe_put_body           (snap *robot_snap,mode_state_struct state,ctrl_struct ctrl);
        void exe_put_leg            (snap *robot_snap,mode_state_struct state,ctrl_struct ctrl);
        void exe_dog                (snap *robot_snap,mode_state_struct state,ctrl_struct ctrl);
        void exe_chair              (snap *robot_snap,mode_state_struct state,ctrl_struct ctrl);
        void exe_lqr                (snap *robot_snap,mode_state_struct state,ctrl_struct ctrl);
        void exe_upstairs           (snap *robot_snap,mode_state_struct state,ctrl_struct ctrl);
        void exe_fall_protect       (snap *robot_snap,mode_state_struct state,ctrl_struct ctrl);
        void exe_off_ground         (snap *robot_snap,mode_state_struct state,ctrl_struct ctrl);
        void exe_get_ground_smooth  (snap *robot_snap,mode_state_struct state,ctrl_struct ctrl);

        void motor_tor_update();
        void motor_rest();
        static std::pair<float32_t,float32_t> roll_feed(float32_t roll_rad, float32_t left_r, float32_t right_r, float32_t target_height);

        mode_state_struct mode_reset();
        mode_state_struct mode_ptr_search();

        using BehaviorFunc = void (app_coordinate::*)(snap*,mode_state_struct,ctrl_struct);
        snap *robot_snap_ptr_ = nullptr;
        static const int16_t size_map = 14+1;
        move_define move_map[size_map] = {
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
        static constexpr BehaviorFunc behavior_table_[E_MODE_CNT] = {
            &app_coordinate::exe_any,
            &app_coordinate::exe_waiting,
            &app_coordinate::exe_put_body,
            &app_coordinate::exe_put_leg,
            &app_coordinate::exe_dog,
            &app_coordinate::exe_chair,
            &app_coordinate::exe_lqr,
            &app_coordinate::exe_upstairs,
            &app_coordinate::exe_fall_protect,
            &app_coordinate::exe_off_ground,
            &app_coordinate::exe_get_ground_smooth
        };
        out_tor motor_output_               = {};
        robot_controller_struct controller_ = {};
        component motor_component_ = {};
        Relay::relay_lqr LQR_target_data = {};
        mode_state_struct mode_state_ = {
            .extern_cmd_ = CMD_EXECUTING,
            .inner_cmd_ = CMD_EXECUTING,
            .current_state_ = E_WAITING,
            .last_state = E_WAITING,
            .reduce_cnt = 0,
            .delta_S = 0,
            .height_record = 0
        };
    };
}



#endif //APP_COORDINATE_H
