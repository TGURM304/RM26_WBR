//
// Created by fish on 2024/12/18.
//

#include "app_chassis.h"

#include "app_LQR.h"
#include "app_coordinate.h"
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
},Motor_Pkg::E_backward,-PI_F32/2);
Motor_Pkg::Joint joint2("join2",Motor::DMMotor::J8009P,{
    .slave_id = 0x12,
    .master_id = 0x02,
    .port = E_CAN2,
    .mode = Motor::DMMotor::MIT,
    .p_max = 3.141593, .v_max = 45, .t_max = 30, .kp_max = 12.5, .kd_max = 5
},Motor_Pkg::E_backward,0)
;Motor_Pkg::Joint joint3("join3",Motor::DMMotor::J8009P,{
    .slave_id = 0x13,
    .master_id = 0x03,
    .port = E_CAN2,
    .mode = Motor::DMMotor::MIT,
    .p_max = 3.141593, .v_max = 45, .t_max = 30, .kp_max = 12.5, .kd_max = 5
},Motor_Pkg::E_forward,0);
Motor_Pkg::Joint joint4("join4",Motor::DMMotor::J8009P,{
    .slave_id = 0x14,
    .master_id = 0x04,
    .port = E_CAN2,
    .mode = Motor::DMMotor::MIT,
    .p_max = 3.141593, .v_max = 45, .t_max = 30, .kp_max = 12.5, .kd_max = 5
},Motor_Pkg::E_forward,-PI_F32/2);
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
auto rc = bsp_rc_data();

INS::app_WRB_ins my_ins(ins);

Relay::StateMapping mapping(&joint1,&joint2,&joint3,&joint4,
    &right_dynamic,&left_dynamic,&my_ins);
Relay::message_adapter adapter(&mapping);
LegController::app_leg_ctrl leg_controller(
{80,2.0/1000,0,30,10},
{50,0,0,5,0},
{1,4.0/1000.0f,0.5,16,12},
{10,0,0,5,3});
LegController::wheel_speed_controller dog_controller({.Kp = 2.5, .Ki = 0.05, .Kd = 0, .out_limit = 2.5, .iout_limit = 0.5});

VMC::app_vmc vmc;
LQR::LQR_controller lqr_controller((float32_t *)K,(float32_t *)K_Fit_Coefficients);
Coordinate::observer_struct my_observer = {
.my_adapter_ = &adapter,
.J1 = &joint1,
.J2 = &joint2,
.J3 = &joint3,
.J4 = &joint4,
.right = &right_dynamic,
.left = &left_dynamic,
.ins = &my_ins
};

Coordinate::snap my_snap(my_observer);
Coordinate::robot_controller_struct my_controller_struct = {
.vmc = &vmc,
.left_vmc_pkg = {},
.right_vmc_pkg = {},
.leg_ctrl = &leg_controller,
.lqr_controller = &lqr_controller,
.dog_ctrl = &dog_controller};
Coordinate::component my_component = {
.j1 = &joint1,
.j2 = &joint2,
.j3 = &joint3,
.j4 = &joint4,
.right = &right_dynamic,
.left = &left_dynamic};
Coordinate::app_coordinate my_coordinate(&my_snap,my_controller_struct,my_component);
// 静态任务，在 CubeMX 中配置
void app_chassis_task(void *args) {
	// Wait for system init.
	while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);
    OS::Task::SleepSeconds(2);
    app_chassis_init();

	while(true) {

        my_coordinate.test_function(rc);

        // TODO: 他妈的为什么没有任何地方调用 dog controller 的 tick
	    dog_controller.tick();

	    auto p = my_snap.current_snap_get();
	    // app_msg_vofa_send(E_UART_DEBUG,
	    //     p->left_leg.theta,
	    //     p->left_leg.dot_theta,
	    //     p->right_leg.theta,
	    //     p->right_leg.dot_theta);

	    app_msg_vofa_send(E_UART_DEBUG, rc->s_l, p->robot_raw_data.speed_left, p->robot_raw_data.speed_right);


     //    my_ins.update();
	    // auto p =my_ins.get_pos();
	    // bsp_uart_printf(E_UART_DEBUG,"%f,%f,%f,%f,%f,%f\r\n",
	    //     p->vector_x,p->vector_y,p->vector_z,
	    //     p->body_roll,p->body_theta,p->body_phi);

	    OS::Task::SleepMilliseconds(1);
	}
}

void app_chassis_init() {
    // my_ins.set_correct(-5.0*3.14/180);
    my_ins.set_correct(0);
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
    my_coordinate.init();
}

#endif