//
// Created by 15082 on 2026/3/22.
//

#include "app_support_clc.h"

#include "robot_data.h"

#include <bits/ranges_base.h>

/**
 * 
 * @param snap 快照系统，用于获取LQR和leg相关信息和
 * @param observer 观测器，用于获取机器人的原始数据
 */
void Coordinate::support::leg_data_update(robot_snap* snap,observer_struct* observer,
    float left_force, float left_tor, float right_force, float right_tor) {
    auto left = &snap->left_leg;
    auto right = &snap->right_leg;
    auto ins = observer->ins;

    float deg_pit, deg_rol, deg_yaw;
    deg_pit= ins->get_pos()->body_theta;
    deg_rol = ins->get_pos()->body_roll;
    deg_yaw = ins->get_pos()->body_phi;
    float acc_ary[3];
    acc_ary[0] = ins->get_pos()->body_x_acc;
    acc_ary[1] = ins->get_pos()->body_y_acc;
    acc_ary[2] = ins->get_pos()->body_z_acc;

    Matrixf<3,1> acc_vector(acc_ary);
    auto z_axis = matrixf::rot_z<3>(deg_yaw);
    auto y_axis = matrixf::rot_y<3>(deg_pit);
    auto x_axis = matrixf::rot_x<3>(deg_rol);
    auto rotation_matrix = z_axis*y_axis*x_axis;
    auto world_acc_vector = rotation_matrix*acc_vector;
    body_z_dot2_ = world_acc_vector[2][0] + G;

    left_data_.L0_dot2_     = left->dot2_L0;
    left_data_.theta_dot2_  = left->dot2_theta;
    left_data_.L0_dot_      = left->dot_L0;
    left_data_.theta_dot    = left->dot_theta;
    left_data_.theta_       = left->theta-PI_F32/2;
    left_data_.L0_          = left->L0;
    left_data_.force_       = left_force;
    left_data_.tor_         = left_tor;

    right_data_.L0_dot2_    = right->dot2_L0;
    right_data_.theta_dot2_ = right->dot2_theta;
    right_data_.L0_dot_     = right->dot_L0;
    right_data_.theta_dot   = right->dot_theta;
    right_data_.theta_      = right->theta-PI_F32/2;
    right_data_.L0_         = right->L0;
    right_data_.force_      = right_force;
    right_data_.tor_        = right_tor;
}

void Coordinate::support::support_clc(leg_switch leg) {
    support_struct* p = nullptr;

    if(leg == E_left)
        p = &left_data_;
    else if(leg == E_right)
        p = &right_data_;

    float32_t dot2_z_w = body_z_dot2_ - p->L0_dot2_
    + 2*p->L0_dot_*sinf(p->theta_) + p->L0_*p->theta_dot2_*sinf(p->theta_)
    + p->L0_*p->theta_dot*p->theta_dot*cosf(p->theta_);

    float32_t force_p = p->force_*cosf(p->theta_) + p->tor_*sinf(p->theta_)/p->L0_;
    float32_t temp = force_p + mess_wheel*G + mess_wheel*dot2_z_w;
    if(leg == E_left)
        force_.left_support_ = temp;
    else if(leg == E_right)
        force_.right_support_ = temp;
}

