//
// Created by 15082 on 2026/1/17.
//

#include "app_observer.h"

#include "bsp_uart.h"
using namespace Relay;
void StateMapping::update() {
    ins_->update();
    motors_.D1 = right_dynamic_->get_status();
    motors_.D2 = left_dynamic_->get_status();
    motors_.J1 = joint1_->get_status();
    motors_.J2 = joint2_->get_status();
    motors_.J3 = joint3_->get_status();
    motors_.J4 = joint4_->get_status();
    //D1和D2会自动更新所以说不用太担心，但是J1~J4由于目前是一问一答模式，要更新状态一定要有控制帧

    leg_clc(motors_.J2.pos,motors_.J1.pos,E_RIGHT);
    leg_clc(motors_.J3.pos,motors_.J4.pos,E_LEFT);

    LQR_status_.body_roll = pos->body_roll;
    LQR_status_.wheel_delta_S += (motors_.D1.pos-motors_.D1.old_pos
        +motors_.D2.pos-motors_.D2.old_pos)/2;
    LQR_status_.wheel_ver = (motors_.D1.speed+motors_.D2.speed)/2;
    LQR_status_.body_phi = pos->body_phi;
    LQR_status_.body_gro = pos->body_phi_gry;
    LQR_status_.body_theta = pos->body_theta;
    LQR_status_.body_dot_theta = pos->body_theta_gry;
    //leg中的theta是相对于机体的腿的角度，看定义atan2(x,y)就可以看出来，但是我们LQR要用的是相对于地面的
    LQR_status_.left_theta = LQR_status_.body_theta+left_leg_status_.theta-PI_F32/2;
    LQR_status_.right_theta = LQR_status_.body_theta+right_leg_status_.theta-PI_F32/2;
    LQR_status_.left_dot_theta = pos->body_theta_gry+left_leg_status_.dot_theta;
    LQR_status_.right_dot_theta = pos->body_theta_gry+right_leg_status_.dot_theta;
}

//theta_big是大腿连杆的角度
//theta_small是小腿连杆的角度
void StateMapping::leg_clc(float theta_big, float theta_small,leg_switch leg){
    leg_status *leg_ptr = &left_leg_status_;
    if(leg == E_LEFT)
        leg_ptr = &left_leg_status_;
    else if(leg == E_RIGHT)
        leg_ptr = &right_leg_status_;
    float theta1, theta2;
    //theta1和theta2是腿上的角度的内容
    theta1 = theta_big;
    float kate_alpha, kate_gama;
    kate_alpha = theta_big - theta_small;
    float kate_c2 = 2*KATE_A*KATE_A*(1-cosf(kate_alpha));
    kate_gama = acosf((2*KATE_B*KATE_B-kate_c2)/(2*KATE_B*KATE_B));
    theta2 = -(kate_alpha+kate_gama)/2;

    leg_ptr->old_L0 = leg_ptr->L0;
    leg_ptr->old_theta = leg_ptr->theta;
    leg_ptr->pos_x = L1*cosf(theta1)+L2*cosf(theta1+theta2);
    leg_ptr->pos_y = L1*sinf(theta1)+L2*sinf(theta1+theta2);
    leg_ptr->L0 = sqrtf(leg_ptr->pos_x*leg_ptr->pos_x+leg_ptr->pos_y*leg_ptr->pos_y);
    leg_ptr->theta = atan2(leg_ptr->pos_y,leg_ptr->pos_x);
    leg_ptr->theta_1 = theta1;
    leg_ptr->theta_2 = theta2;
    if(leg == E_LEFT) {
        left_filter_.input(leg_ptr->theta - leg_ptr->old_theta);
        leg_ptr->dot_theta = left_filter_.get()*1000;
        left_L0_filter_.input(leg_ptr->L0 - leg_ptr->old_L0);
        leg_ptr->dot_L0 = left_L0_filter_.get()*1000;
    }
    else if(leg == E_RIGHT) {
        right_filter_.input(leg_ptr->theta - leg_ptr->old_theta);
        leg_ptr->dot_theta = right_filter_.get()*1000;
        right_L0_filter_.input(leg_ptr->L0 - leg_ptr->old_L0);
        leg_ptr->dot_L0 = right_L0_filter_.get()*1000;
    }
}

void StateMapping::clear_s(){
    LQR_status_.wheel_delta_S = 0;
}

