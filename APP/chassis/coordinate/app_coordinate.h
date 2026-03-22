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
#include "app_dog_ctrl.h"
#include "app_ibc.h"
#include "app_behavior_define.h"
#include "app_support_clc.h"

#include <vector>

#define REDUCE_EDGE_DOWN   300
#define REDUCE_EDGE_UP    500
#define DELTA_S_EDGE    0.40f
#define DELTA_VER_EDGE  2.5f

#define FILTER_ORDER 2
#define FILTER_FC_HIGH 100.0f
#define FILTER_FC_LOW 10.0f
#define FILTER_FS 1000.0f

#define VX_MAX 3.0f
#define VX_MIN (-3.0f)
#define VY_MAX 3.0f
#define VY_MIN (-3.0f)
#define SPIN_RAD_MAX (6.0f)
#define SPIN_RAD_MIN (-6.0f)



namespace Coordinate {
    class app_coordinate {
    public:
        app_coordinate();
        app_coordinate(snap* robot_snap, robot_controller_struct controller, component motor_component)
        :robot_snap_ptr_(robot_snap), controller_(controller),
        motor_component_(motor_component),ibc_gimbal_(E_CAN3,GIMBAL_ID),
        support_(30)
         {
            mode_state_ = {
                .extern_cmd_ = CMD_EXECUTING,
                .inner_cmd_ = CMD_EXECUTING,
                .current_state_ = E_WAITING,
                .last_state = E_WAITING,
                .reduce_cnt = 0,
                .delta_S = 0,
                .height_record = 0
            };
        };
        void tick();
        void init();
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

        void ibc_send_update(float vector_x, float vector_y, float vector_z, mode_state mode);
        static std::pair<float32_t,float32_t> roll_feed(float32_t roll_rad, float32_t left_r, float32_t right_r, float32_t target_height);
        mode_state_struct mode_ptr_search();
        using BehaviorFunc = void (app_coordinate::*)(snap*,mode_state_struct,ctrl_struct);

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
        out_tor motor_output_ = {};
        robot_controller_struct controller_ = {};
        component motor_component_ = {};
        Relay::relay_lqr LQR_target_data = {};
        mode_state_struct mode_state_ = {};
        snap *robot_snap_ptr_ = nullptr;

        uint16_t send_cnt;
        IBC::chassis chassis_send_;
        app_msg_can_receiver<IBC::gimbal> ibc_gimbal_;
        support support_;
    };
}



#endif //APP_COORDINATE_H
