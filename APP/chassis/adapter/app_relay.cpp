//
// Created by 15082 on 2026/1/24.
//

#include "app_relay.h"

void Relay::message_adapter::update() {
    mapping_->update();
    //对腿的状态进行更新
    auto p = &LQR_raw_data_;
    p->S += mapping_->get_lqr_status().wheel_delta_S;
    p->dot_S = mapping_->get_lqr_status().wheel_ver;
    p->phi = mapping_->get_lqr_status().body_phi;
    p->dot_phi = mapping_->get_lqr_status().body_gro;
    p->left_theta = mapping_->get_lqr_status().left_theta;
    p->left_dot_theta = mapping_->get_lqr_status().left_dot_theta;
    p->right_theta = mapping_->get_lqr_status().right_theta;
    p->right_dot_theta = mapping_->get_lqr_status().right_dot_theta;
    p->body_theta = mapping_->get_lqr_status().body_theta;
    p->body_dot_theta = mapping_->get_lqr_status().body_dot_theta;
    p->data_x[0] = p->S;
    p->data_x[1] = p->dot_S;
    p->data_x[2] = p->phi;
    p->data_x[3] = p->dot_phi;
    p->data_x[4] = p->left_theta;
    p->data_x[5] = p->left_dot_theta;
    p->data_x[6] = p->right_theta;
    p->data_x[7] = p->right_dot_theta;
    p->data_x[8] = p->body_theta;
    p->data_x[9] = p->body_dot_theta;
    mapping_->clear_s();

    auto leg = mapping_->get_leg_status(Relay::E_LEFT);
    left_leg_status_.L0 = leg.L0;
    left_leg_status_.dot_L0 = leg.dot_L0;
    left_leg_status_.theta = leg.theta;
    left_leg_status_.dot_theta = leg.dot_theta;
    left_leg_status_.theta_1 = leg.theta_1;
    left_leg_status_.theta_2 = leg.theta_2;
    leg = mapping_->get_leg_status(Relay::E_RIGHT);
    right_leg_status_.L0 = leg.L0;
    right_leg_status_.dot_L0 = leg.dot_L0;
    right_leg_status_.theta = leg.theta;
    right_leg_status_.dot_theta = leg.dot_theta;
    right_leg_status_.theta_1 = leg.theta_1;
    right_leg_status_.theta_2 = leg.theta_2;
}

Relay::relay_lqr Relay::message_adapter::get_LQR_raw() {
    return LQR_raw_data_;
}

Relay::relay_leg Relay::message_adapter::get_left_leg_status() {
    return left_leg_status_;
}

Relay::relay_leg Relay::message_adapter::get_right_leg_status() {
    return right_leg_status_;
}

void Relay::message_adapter::clear() {
    mapping_->clear_s();
    LQR_raw_data_ = {};
    left_leg_status_ = {};
    right_leg_status_ = {};
}
