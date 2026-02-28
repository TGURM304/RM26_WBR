//
// Created by 15082 on 2026/2/12.
//

#include "app_snap.h"

Coordinate::robot_snap* Coordinate::snap::current_snap_get() {
    return &current_snap_;
}

Coordinate::robot_snap* Coordinate::snap::last_snap_get() {
    return &last_snap_;
}

Coordinate::robot_snap *Coordinate::snap::zero_snap_get() {
    return &zero_snap_;
}


void Coordinate::snap::snap_update() {
    memcpy(&last_snap_,&current_snap_,sizeof(current_snap_));
    auto p = observer_.my_adapter_;
    observer_.my_adapter_->update();
    current_snap_.lqr_data = p->get_LQR_raw();
    current_snap_.left_leg = p->get_left_leg_status();
    current_snap_.right_leg = p->get_right_leg_status();
    auto raw = &current_snap_.robot_raw_data;

    raw->speed_left = observer_.left->get_status().speed;
    raw->dis_left += raw->speed_left/1000;
    raw->speed_right = observer_.right->get_status().speed;
    raw->dis_right += raw->speed_right/1000;

    raw->deg_J1 = observer_.J1->get_status().pos;
    raw->deg_J2 = observer_.J2->get_status().pos;
    raw->deg_J3 = observer_.J3->get_status().pos;
    raw->deg_J4 = observer_.J4->get_status().pos;

    observer_.ins->update();
    raw->body_theta = observer_.ins->get_pos()->body_theta;
    raw->body_phi = observer_.ins->get_pos()->body_phi;
    raw->body_roll = observer_.ins->get_pos()->body_roll;

    current_snap_.state_flag |= SNAP_START;

    raw->vector_x = observer_.ins->get_pos()->vector_x;
    raw->vector_y = observer_.ins->get_pos()->vector_y;
    raw->vector_z = observer_.ins->get_pos()->vector_z;
}

void Coordinate::snap::snap_clear() {
    observer_.my_adapter_->clear();
    current_snap_ = {};
    zero_snap_ = {};
    memcpy(&last_snap_,&current_snap_,sizeof(current_snap_));
}

void Coordinate::snap::snap_set_zero() {
    zero_snap_.robot_raw_data.body_phi = current_snap_.robot_raw_data.body_phi;
    zero_snap_.lqr_data.S = current_snap_.lqr_data.S;
    zero_snap_.lqr_data.phi = current_snap_.lqr_data.phi;
}

void Coordinate::snap::snap_clear_S() {
    observer_.my_adapter_->clear_S();
    current_snap_.lqr_data.S = 0;
    zero_snap_.lqr_data.S = 0;
}
