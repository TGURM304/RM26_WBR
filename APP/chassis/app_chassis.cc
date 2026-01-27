//
// Created by fish on 2024/12/18.
//

#include "app_chassis.h"

#include "app_msg.h"

#include <app_leg_ctrl.h>
#include <app_relay.h>


#ifdef COMPILE_CHASSIS

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
Controller::app_leg_ctrl leg_controller(
    {20,0,2,10,10},
    {50,0,0,3,5},
    {3,0,2,10,10},
    {10,0,0,3,3});
VMC::app_vmc vmc;
VMC::ctrl_pkg ctrl_pkg_right = {0};
// 静态任务，在 CubeMX 中配置
void app_chassis_task(void *args) {
	// Wait for system init.
	while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);
    OS::Task::SleepSeconds(2);
    app_chassis_init();
    auto tor = vmc.tor_get();
    ctrl_pkg_right.force_x = 0;
    ctrl_pkg_right.force_y = 0;
    ctrl_pkg_right.force_L = 0;
    ctrl_pkg_right.leg_tor = 0;
	while(true) {
	    joint1.set_tor(tor.p_right_tor2);
	    joint2.set_tor(tor.p_right_tor1);
	    joint3.set_tor(0);
	    joint4.set_tor(0);
	    right_dynamic.set_tor(0);
	    left_dynamic.set_tor(0);
        adapter.update();
	    auto right_leg = adapter.get_right_leg_status();
	    leg_controller.right_len_update(right_leg,0.25);
	    leg_controller.right_deg_update(right_leg,PI_F32/2);
	    ctrl_pkg_right.force_L = leg_controller.get_output().force_right;
	    ctrl_pkg_right.leg_tor = leg_controller.get_output().tor_right;
	    vmc.tor_clc(ctrl_pkg_right,right_leg,VMC::E_Right);
	    tor = vmc.tor_get();
	    auto leg = mapping.get_leg_status(Relay::E_RIGHT);
	    OS::Task::SleepMilliseconds(1);
	}
}

void app_chassis_init() {
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