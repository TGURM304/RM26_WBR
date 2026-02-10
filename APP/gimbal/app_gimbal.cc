//
// Created by fish on 2024/11/17.
//

#include "app_gimbal.h"

#include "app_head.h"
#include "bsp_uart.h"
#include "dev_motor_dm.h"

#ifdef COMPILE_GIMBAL

#define MOTOR_ZERO 6640

MotorController left_shoot(std::make_unique <Motor::DJIMotor>(
    "left_shoot", Motor::DJIMotor::M3508,
    (Motor::DJIMotor::Param) {.id = 0x02,.port = E_CAN2,.mode = Motor::DJIMotor::CURRENT}
));
MotorController right_shoot(std::make_unique <Motor::DJIMotor>(
    "right_shoot", Motor::DJIMotor::M3508,
    (Motor::DJIMotor::Param) {.id = 0x01,.port = E_CAN2,.mode = Motor::DJIMotor::CURRENT}
));
Motor::DJIMotor yaw_motor("yaw_motor",Motor::DJIMotor::GM6020,
    {.id = 0x01, .port = E_CAN3, .mode = Motor::DJIMotor::CURRENT});
Motor::DMMotor pit_motor("pit_motor",Motor::DMMotor::J4310,
    {.slave_id = 0x21, .master_id = 0x11, .port = E_CAN1,
        .mode = Motor::DMMotor::MIT, .p_max = 12.5, .v_max = 30, .t_max = 10,
        .kp_max = 500, .kd_max = 5});

Controller::PID pit_pos(0,0,0,0,0);
Controller::PID pit_speed(0,0,0,0,0);
Controller::PID yaw_pos(8,0,0,8,0);
Controller::PID yaw_speed(0.25,2.0/1000,0,3,0.5);

Gimbal::Head my_head(&yaw_motor,&pit_motor);

auto ins = app_ins_data();
auto rc = bsp_rc_data();

void app_gimbal_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready())
        OS::Task::SleepMilliseconds(10);
    app_gimbal_init();
    yaw_motor.init();
    yaw_motor.enable();
    pit_motor.init();
    pit_motor.enable();
    while(true) {

        my_head.head_update();
        my_head.head_pid_clc(((float)(rc->rc_r[0]))*3/640.0f,0);
        if(rc->s_l == 1) my_head.head_output();
        else my_head.head_relax();
        // bsp_uart_printf(E_UART_DEBUG,"%f,%f,%f,%f\r\n",
        //     pit_motor.status.pos,yaw_motor.status.angle,
        //     yaw_motor.status.current,yaw_motor.status.speed);
        // bsp_uart_printf(E_UART_DEBUG,"%f,%f,%f,%f\r\n"
        //     , my_head.gimbal_pkg_.yaw_current,
        //     my_head.pid_yaw_out_,
        //     my_head.gimbal_pkg_.yaw_speed,
        //     (float)(rc->rc_r[0])*2/640.0f);
        OS::Task::SleepMilliseconds(1);
    }
}

void app_gimbal_init() {
    left_shoot.add_controller(std::make_unique <Controller::MotorBasePID> (
        Controller::MotorBasePID::PID_SPEED,
        std::make_unique <Controller::PID> (5.0, 1.0/1000, 0.0, 16384, 10000),
        nullptr
        ));
    right_shoot.add_controller(std::make_unique <Controller::MotorBasePID> (
            Controller::MotorBasePID::PID_SPEED,
            std::make_unique <Controller::PID> (5.0, 1.0/1000, 0.0, 16384, 10000),
            nullptr
        ));
    left_shoot.init();
    right_shoot.init();

    my_head.head_init(yaw_pos,yaw_speed,
        pit_pos,pit_speed);
}

#endif
