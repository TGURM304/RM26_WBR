//
// Created by fish on 2024/12/18.
//

#include "app_chassis.h"

#include "app_LQR.h"
#include "app_control_pipeline.h"
#include "app_msg.h"
#include "bsp_rc.h"

#include <app_leg_ctrl.h>
#include <app_relay.h>
#include "lqr_matrix.h"

#include <ios>

#ifdef COMPILE_CHASSIS

#define LIMIT(max,data) (((data) > (max)) ? (max) : (((data) < -(max)) ? -(max) : (data)))

//电机对象定义
Motor_Pkg::Joint joint1("join1",Motor::DMMotor::J8009P,{
  .slave_id = 0x11,
  .master_id = 0x01,
  .port = E_CAN2,
  .mode = Motor::DMMotor::MIT,
  .p_max = 3.141593, .v_max = 45, .t_max = 30, .kp_max = 12.5, .kd_max = 5
},Motor_Pkg::E_backward,0);
Motor_Pkg::Joint joint2("join2",Motor::DMMotor::J8009P,{
    .slave_id = 0x12,
    .master_id = 0x02,
    .port = E_CAN2,
    .mode = Motor::DMMotor::MIT,
    .p_max = 3.141593, .v_max = 45, .t_max = 30, .kp_max = 12.5, .kd_max = 5
},Motor_Pkg::E_backward,PI_F32/2);Motor_Pkg::Joint joint3("join3",Motor::DMMotor::J8009P,{
    .slave_id = 0x13,
    .master_id = 0x03,
    .port = E_CAN2,
    .mode = Motor::DMMotor::MIT,
    .p_max = 3.141593, .v_max = 45, .t_max = 30, .kp_max = 12.5, .kd_max = 5
},Motor_Pkg::E_forward,PI_F32/2);
Motor_Pkg::Joint joint4("join4",Motor::DMMotor::J8009P,{
    .slave_id = 0x14,
    .master_id = 0x04,
    .port = E_CAN2,
    .mode = Motor::DMMotor::MIT,
    .p_max = 3.141593, .v_max = 45, .t_max = 30, .kp_max = 12.5, .kd_max = 5
},Motor_Pkg::E_forward,0);
Motor_Pkg::Dynamic right_dynamic("right_dynamic",Motor::DJIMotor::M3508,{
    .id = 1,
    .port = E_CAN1,
    .mode = Motor::DJIMotor::CURRENT
},Motor_Pkg::E_forward,268.0f/17.0f,WHEEL_R);
Motor_Pkg::Dynamic left_dynamic("left_dynamic",Motor::DJIMotor::M3508,{
    .id = 2,
    .port = E_CAN1,
    .mode = Motor::DJIMotor::CURRENT
},Motor_Pkg::E_backward,268.0f/17.0f,WHEEL_R);


const app_ins_data_t *ins = app_ins_data();
INS::app_WRB_ins my_ins(ins);

Relay::StateMapping mapping(&joint1,&joint2,&joint3,&joint4,
    &right_dynamic,&left_dynamic,&my_ins);
Relay::message_adapter adapter(&mapping);
LegController::app_leg_ctrl leg_controller(
{70,3.0/1000,2,15,10},
{80,0,0,3,5},
{3,0,2,10,10},
{10,0,0,3,3});
VMC::app_vmc vmc;
LQR::LQR_controller lqr_controller((float32_t *)K,(float32_t *)coef);
VMC::ctrl_pkg left_vmc_pkg, right_vmc_pkg;
auto rc = bsp_rc_data();

// 静态任务，在 CubeMX 中配置
void app_chassis_task(void *args) {
	// Wait for system init.
	while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);
    OS::Task::SleepSeconds(2);
    app_chassis_init();
    Pipeline::final_output motor_tor;
    float32_t delta[10] = {0};
    float32_t target_s = 0;
	while(true) {
	    adapter.update();
	    auto left_leg = adapter.get_left_leg_status();
	    auto right_leg = adapter.get_right_leg_status();
	    auto lqr_data = adapter.get_LQR_raw();

	    leg_controller.left_len_update(left_leg,0.22);
	    leg_controller.right_len_update(right_leg,0.22);
	    leg_controller.left_deg_update(left_leg,PI/2);
	    leg_controller.right_deg_update(right_leg,PI/2);
	    auto pid_out = leg_controller.get_output();

        left_vmc_pkg.force_L = pid_out.force_left;
	    left_vmc_pkg.leg_tor = pid_out.tor_left;


	    right_vmc_pkg.force_L = pid_out.force_right;
	    right_vmc_pkg.leg_tor = pid_out.tor_right;


	    vmc.tor_clc(left_vmc_pkg,left_leg,VMC::E_Left);
	    vmc.tor_clc(right_vmc_pkg,right_leg,VMC::E_Right);
	    auto vmc_out = vmc.tor_get();

	    if(rc->s_r == 1) {
	        target_s += (float32_t)rc->rc_l[1]*0.5/660/1000;

	        delta[0] = target_s -lqr_data.S;
	        delta[1] = -lqr_data.dot_S;
	        delta[0] = LIMIT(1,delta[0]);

	        delta[2] = -lqr_data.phi;
	        delta[3] = -lqr_data.dot_phi;
	        // delta[4] = -lqr_data.left_theta;
	        // delta[5] = -lqr_data.left_dot_theta;
	        // delta[6] = -lqr_data.right_theta;
	        // delta[7] = -lqr_data.right_dot_theta;
	        delta[8] = -lqr_data.body_theta;
	        delta[9] = -lqr_data.body_dot_theta;
	        // delta[8] = LIMIT(0.5,delta[8]);


	        // delta[0] = 0;
	        // delta[1] = 0;
	        // delta[2] = 0;
	        // delta[3] = 0;
	        delta[4] = 0;
	        delta[5] = 0;
	        delta[6] = 0;
	        delta[7] = 0;
	        // delta[8] = 0;
	        // delta[9] = 0;
	        lqr_controller.static_clc(delta);
	        auto left = lqr_controller.get_lqr_output(LQR::E_left);
	        auto right = lqr_controller.get_lqr_output(LQR::E_right);

	        joint1.set_tor(vmc_out.p_right_tor2+vmc_out.c_right_tor2);
	        joint2.set_tor(vmc_out.p_right_tor1+vmc_out.c_right_tor1);
	        joint3.set_tor(vmc_out.p_left_tor1+vmc_out.c_left_tor1);
	        joint4.set_tor(vmc_out.p_left_tor2+vmc_out.c_left_tor2);
	        if(rc->s_l == 1) {
	            left_vmc_pkg.force_y = 40*cos(lqr_data.body_theta);
	            left_vmc_pkg.force_x = -40*sin(lqr_data.body_theta);
	            right_vmc_pkg.force_y = 40*cos(lqr_data.body_theta);
	            right_vmc_pkg.force_x = -40*sin(lqr_data.body_theta);
	            left_dynamic.set_tor(left.wheel_move+left.wheel_balance);
                right_dynamic.set_tor(right.wheel_move+right.wheel_balance);
                // left_dynamic.set_tor(left.body_balance);
                // right_dynamic.set_tor(right.body_balance);
                // left_dynamic.set_tor(0);
                // right_dynamic.set_tor(0);
                // left_dynamic.set_tor(left.body_move);
	            // right_dynamic.set_tor(right.body_move);
	            // left_dynamic.set_tor((float)(rc->rc_l[1]*3.0f)/640.f);
	            // right_dynamic.set_tor((float)(rc->rc_l[1]*3.0f)/640.f);

                bsp_uart_printf(E_UART_DEBUG,"%f,%f,%f,%f,%f,%f\r\n",
	            left.wheel_move,right.wheel_move,
	            left.wheel_balance,left.wheel_move+left.wheel_balance,
	            delta[0],delta[8]);
	        }
	        else {
	            target_s = 0;
	            left_vmc_pkg.force_y = 0;
	            left_vmc_pkg.force_x = 0;
	            right_vmc_pkg.force_y = 0;
	            right_vmc_pkg.force_x = 0;
	            left_dynamic.set_tor(0);
	            right_dynamic.set_tor(0);
	        }
	    }
        else {
            target_s = 0;
            joint1.set_tor(0);
            joint2.set_tor(0);
            joint3.set_tor(0);
            joint4.set_tor(0);
            right_dynamic.set_tor(0);
            left_dynamic.set_tor(0);
            leg_controller.leg_clear();
            adapter.clear();
        }

	    OS::Task::SleepMilliseconds(1);
	}
}

void app_chassis_init() {
    my_ins.set_correct(-5.0*3.14/180);
    right_dynamic.pkg_init();
    left_dynamic.pkg_init();
    joint1.pkg_init(), joint2.pkg_init(), joint3.pkg_init(), joint4.pkg_init();
    joint1.pkg_reset();
    OS::Task::SleepMilliseconds(10);
    joint2.pkg_reset();
    OS::Task::SleepMilliseconds(10);
    joint3.pkg_reset();
    OS::Task::SleepMilliseconds(10);
    joint4.pkg_reset();
    OS::Task::SleepMilliseconds(10);
    // while(joint1.get_status().err != 1)
        joint1.pkg_enable(), OS::Task::SleepMilliseconds(10);
    // while(joint2.get_status().err != 1)
        joint2.pkg_enable(), OS::Task::SleepMilliseconds(10);
    // while(joint3.get_status().err != 1)
        joint3.pkg_enable(), OS::Task::SleepMilliseconds(10);
    // while(joint4.get_status().err != 1)
        joint4.pkg_enable(), OS::Task::SleepMilliseconds(10);
}

#endif