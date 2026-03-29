//
// Created by fish on 2024/11/17.
//

#include "app_gimbal.h"

#include "app_cmd.h"
#include "app_gimbal_coordinate.h"
#include "app_shoot_controller.h"
#include "app_head_controller.h"
#include "bsp_uart.h"
#include "dev_motor_dm.h"
#include "app_second_order.h"
#include "app_vision_core.h"
#include "robomaster.h"
#include "const_data/robot_data.h"


#ifdef COMPILE_GIMBAL

#define MOTOR_ZERO 6640


Motor::DJIMotor left_shoot_motor("left_shoot_motor",
                                 Motor::DJIMotor::M3508,
                                 (Motor::DJIMotor::Param){
                                     .id = 0x02, .port = E_CAN2, .mode = Motor::DJIMotor::CURRENT });
Motor::DJIMotor right_shoot_motor("right_shoot_motor",
                                  Motor::DJIMotor::M3508,
                                  (Motor::DJIMotor::Param){
                                      .id = 0x01, .port = E_CAN2, .mode = Motor::DJIMotor::CURRENT });
Motor::DJIMotor trigger_motor("trigger_motor",
                              Motor::DJIMotor::M3508,
                              (Motor::DJIMotor::Param){ .id = 0x01, .port = E_CAN3, .mode = Motor::DJIMotor::CURRENT });
Motor::DJIMotor
    yaw_motor("yaw_motor", Motor::DJIMotor::GM6020, { .id = 0x01, .port = E_CAN3, .mode = Motor::DJIMotor::CURRENT });
Motor::DMMotor pit_motor("pit_motor",
                         Motor::DMMotor::J4310,
                         { .slave_id  = 0x21,
                           .master_id = 0x11,
                           .port      = E_CAN1,
                           .mode      = Motor::DMMotor::MIT,
                           .p_max     = 12.5,
                           .v_max     = 30,
                           .t_max     = 10,
                           .kp_max    = 500,
                           .kd_max    = 5 });

auto ins = app_ins_data();
auto rc  = bsp_rc_data();


Gimbal::snap robo_snap(
    &yaw_motor, &pit_motor, &trigger_motor, &left_shoot_motor, &right_shoot_motor, ins, PITCH_ZERO_POINT, 0.0f);
Gimbal::Head my_head(&yaw_motor, &pit_motor, &robo_snap);
Gimbal::Shoot my_shoot(&left_shoot_motor, &right_shoot_motor, &trigger_motor, &robo_snap);
Gimbal::vision_core my_vision_core(1000, 60, &robo_snap);
Gimbal::gimbal_coordinate coordinate(&my_shoot,&my_head,&my_vision_core);
void app_gimbal_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready())
        OS::Task::SleepMilliseconds(10);
    yaw_motor.init();
    yaw_motor.enable();
    pit_motor.init();
    pit_motor.enable();

    while(true) {
        robo_snap.snap_update();
        coordinate.update_rc(rc);

        coordinate.tick();
        bsp_uart_printf(E_UART_1,"%f,%f\r\n",
            coordinate.ctrl_.get_cmd()->delta_pit,
            coordinate.ctrl_.get_cmd()->delta_head_yaw);

        OS::Task::SleepMilliseconds(1);
    }
}

void app_gimbal_init() {
    left_shoot_motor.init();
    left_shoot_motor.enable();
    right_shoot_motor.init();
    right_shoot_motor.enable();
    trigger_motor.init();
    trigger_motor.enable();

    coordinate.init();
    robo_snap.snap_init();
}

#endif
