//
// Created by 15082 on 2026/1/17.
//

#ifndef APP_OBSERVER_H
#define APP_OBSERVER_H
#include "app_WRB_ins.h"
#include "app_midaverage_filter.h"
#include "app_motor_pkg.h"


#include <atomic>

namespace Observer {
using namespace Motor_Pkg;
#define L1 210.0f
#define L2 250.0f
#define KATE_A 94.5f
#define KATE_B 112.5f
    typedef struct {
        float body_roll; //用于姿态控制
        float body_S, body_ver;//用于LQR的位移S和速度dot_S
        float body_phi, body_gro;//用于LQR的偏航角yaw和dot_yaw
        float left_theta,left_dot_theta;
        float right_theta, right_dot_theta;//用于LQR的左右腿的倾角theta和dot_theta
        float body_theta, body_dot_theta;//用于LQR的机体的倾角theta和dot_theta
    }car_status;
    typedef struct {
        Motor_Pkg::motor_status_pkg D1,D2;
        Motor_Pkg::motor_status_pkg J1,J2,J3,J4;
    }motor_status;
    typedef struct {
        float pos_x, pos_y;
        float theta, dot_theta;
        float old_theta;
        float L0;
    }leg_status;
    typedef enum {
        E_LEFT,
        E_RIGHT
    }leg_switch;
    class StateMapping {
    public:
        StateMapping(Joint *J1, Joint *J2, Joint *J3, Joint *J4, Dynamic *D1, Dynamic *D2, INS::app_WRB_ins *ins)
        :joint1_(J1),joint2_(J2),joint3_(J3),joint4_(J4),right_dynamic_(D1),left_dynamic_(D2), ins_(ins) {
            pos = ins_->get_pos();
        }
        void update();
    private:
        void leg_clc(float J1_theta, float J2_theta,leg_switch leg);
        Motor_Pkg::Dynamic *left_dynamic_;
        Motor_Pkg::Dynamic *right_dynamic_;
        Motor_Pkg::Joint *joint1_;
        Motor_Pkg::Joint *joint2_;
        Motor_Pkg::Joint *joint3_;
        Motor_Pkg::Joint *joint4_;
        INS::app_WRB_ins *ins_;
        const INS::body_posture  *pos;
        motor_status motors_;
        car_status LQR_status_;
        leg_status left_leg_status_;
        leg_status right_leg_status_;

        TrimmedMeanFilter<10,float32_t> left_filter_;
        TrimmedMeanFilter<10,float32_t> right_filter_;
    };
} // Observer

#endif
