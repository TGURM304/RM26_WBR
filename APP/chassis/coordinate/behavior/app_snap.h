//
// Created by 15082 on 2026/2/12.
//

#ifndef APP_SNAP_H
#define APP_SNAP_H
#include "app_relay.h"

namespace Coordinate {
typedef struct {
    Relay::message_adapter *my_adapter_;
    Motor_Pkg::Joint *J1;
    Motor_Pkg::Joint *J2;
    Motor_Pkg::Joint *J3;
    Motor_Pkg::Joint *J4;
    Motor_Pkg::Dynamic *right;
    Motor_Pkg::Dynamic *left;
    INS::app_WRB_ins *ins;
}observer_struct;
#define SNAP_START (0x01)
typedef struct {
    float deg_J1, deg_J2, deg_J3, deg_J4;
    float speed_right, speed_left;
    float dis_right, dis_left;
    float body_theta, body_phi, body_roll;
}raw_data;
typedef struct {
    uint8_t state_flag;
    Relay::relay_leg left_leg;
    Relay::relay_leg right_leg;
    Relay::relay_lqr lqr_data;
    raw_data robot_raw_data;
}robot_snap;
    class snap {
    public:
        snap();
        snap(observer_struct observer):observer_(observer) {

        }
        void snap_update();
        robot_snap* current_snap_get();
        robot_snap* last_snap_get();
        void snap_clear();
        void snap_set_zero();
    private:
        observer_struct observer_ = {};

        robot_snap current_snap_ = {};
        robot_snap zero_snap_ = {};
        robot_snap last_snap_ = {};
    };
}





#endif //APP_SNAP_H
