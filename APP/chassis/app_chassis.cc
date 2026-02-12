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
Pipeline::control_pipeline my_pipeline(&adapter, &vmc, &lqr_controller,&leg_controller);
Pipeline::cmd_pkg pipeline_pkg = {};

// 静态任务，在 CubeMX 中配置
void app_chassis_task(void *args) {
	// Wait for system init.
	while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);
    OS::Task::SleepSeconds(2);
    app_chassis_init();
    Pipeline::final_output motor_tor;
    float32_t delta[10] = {0};
    float32_t target_s = 0;
    my_pipeline.rest();

	while(true) {
        my_pipeline.observer_update();
	    if(rc->s_r == 1) {
	        pipeline_pkg.leg_deg_cmd = Pipeline::E_LEG_DEG_ENABLE;
	        pipeline_pkg.leg_len_cmd = Pipeline::E_LEG_LEN_ENABLE;
	        if(rc->s_l == 1) {
	            pipeline_pkg.leg_forward_cmd = Pipeline::E_LEG_FORWARD_ENABLE;
	            pipeline_pkg.lqr_cmd = Pipeline::E_LQR_CHAIR;
	        }
	        else {
	            pipeline_pkg.leg_forward_cmd = Pipeline::E_LEG_FORWARD_DISABLE;
                pipeline_pkg.lqr_cmd = Pipeline::E_CHASSIS_SAFE;
	        }
	    }
	    else {
	        my_pipeline.data_clear();
	        pipeline_pkg.leg_deg_cmd = Pipeline::E_LEG_DEG_DISABLE;
	        pipeline_pkg.leg_len_cmd = Pipeline::E_LEG_LEN_DISABLE;
	        pipeline_pkg.leg_forward_cmd = Pipeline::E_LEG_FORWARD_DISABLE;
	    }
	    my_pipeline.set_state(pipeline_pkg);
        my_pipeline.leg_len_control(0.21,0.21);
	    my_pipeline.leg_deg_control(PI/2,PI/2);
        my_pipeline.vmc_pkg_update();
	    my_pipeline.vmc_clc();
	    my_pipeline.motor_tor_update();
	    my_pipeline.lqr_control(rc->rc_l[1]*0.5f/660,(float)rc->reserved/660.0f);
	    my_pipeline.lqr_get();
	    if(rc->s_r == 1) {
	        joint1.set_tor(my_pipeline.output_.tor1);
	        joint2.set_tor(my_pipeline.output_.tor2);
	        joint3.set_tor(my_pipeline.output_.tor3);
	        joint4.set_tor(my_pipeline.output_.tor4);
	        if(rc->s_l == 1) {
	            left_dynamic.set_tor(my_pipeline.output_.dynamic_left);
	            right_dynamic.set_tor(my_pipeline.output_.dynamic_right);
	        }
	        else {
	            left_dynamic.set_tor(0);
	            right_dynamic.set_tor(0);
	        }
	    }
	    else {
	        joint1.set_tor(0);
	        joint2.set_tor(0);
	        joint3.set_tor(0);
	        joint4.set_tor(0);
	        left_dynamic.set_tor(0);
	        right_dynamic.set_tor(0);
	    }
	    bsp_uart_printf(E_UART_DEBUG,"%f,%f,%f,%f,%f\r\n",
	        my_pipeline.left_leg_.theta,
	        my_pipeline.lqr_left.wheel_balance,
	        my_pipeline.lqr_left.wheel_move,
	        my_pipeline.lqr_right.wheel_balance,
            (float)rc->reserved);

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