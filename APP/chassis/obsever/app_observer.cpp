//
// Created by 15082 on 2026/1/17.
//

#include "app_observer.h"
using namespace Observer;
void StateMapping::update() {
    ins_->update();
    motors_.D1 = right_dynamic_->get_status();
    motors_.D2 = left_dynamic_->get_status();
    motors_.J1 = joint1_->get_status();
    motors_.J2 = joint2_->get_status();
    motors_.J3 = joint3_->get_status();
    motors_.J4 = joint4_->get_status();
    //D1和D2会自动更新所以说不用太担心，但是J1~J4由于目前是一问一答模式，要更新状态一定要有控制帧

    leg_clc(motors_.J1.pos,motors_.J2.pos,E_RIGHT);
    leg_clc(motors_.J4.pos,motors_.J3.pos,E_LEFT);

    LQR_status_.body_roll = pos->body_phi;
    LQR_status_.body_S += (motors_.D1.pos-motors_.D1.old_pos
        +motors_.D2.pos-motors_.D2.old_pos)/2;
    LQR_status_.body_ver = (motors_.D1.speed+motors_.D2.speed)/2;
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


void StateMapping::leg_clc(float J1_theta, float J2_theta,leg_switch leg){
    leg_status *leg_ptr = &left_leg_status_;
    if(leg == E_LEFT)
        leg_ptr = &left_leg_status_;
    else if(leg == E_RIGHT)
        leg_ptr = &right_leg_status_;
    float theta1, theta2;
    //theta1和theta2是腿上的角度的内容
    theta1 = J1_theta;
    float kate_alpha, kate_gama;
    kate_alpha = J1_theta - J2_theta;
    float kate_c2 = 2*KATE_A*KATE_A*(1-cosf(kate_alpha));
    kate_gama = acosf((2*KATE_B*KATE_B-kate_c2)/(2*KATE_B*KATE_B));
    theta2 = -(kate_alpha+kate_gama)/2;

    leg_ptr->old_theta = leg_ptr->theta;
    leg_ptr->pos_x = L1*cosf(theta1)+L2*cosf(theta1+theta2);
    leg_ptr->pos_y = L1*sinf(theta1)+L2*sinf(theta1+theta2);
    leg_ptr->L0 = sqrtf(leg_ptr->pos_x*leg_ptr->pos_x+leg_ptr->pos_y*leg_ptr->pos_y);
    leg_ptr->theta = atan2(leg_ptr->pos_y,leg_ptr->pos_x);
    if(leg == E_LEFT) {
        left_filter_.input(leg_ptr->theta - leg_ptr->old_theta);
        leg_ptr->dot_theta = left_filter_.get();
    }
    else if(leg == E_RIGHT) {
        right_filter_.input(leg_ptr->theta - leg_ptr->old_theta);
        leg_ptr->dot_theta = right_filter_.get();
    }
}
