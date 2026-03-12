//
// Created by fish on 2024/11/17.
//

#include "app_gimbal.h"

#include "app_shoot_controller.h"
#include "app_head_controller.h"
#include "bsp_uart.h"
#include "dev_motor_dm.h"
#include "app_second_order.h"
#include "app_vision_core.h"

#include <app_vision.h>
#include <fast_math_functions.h>

#ifdef COMPILE_GIMBAL

#define MOTOR_ZERO 6640


Motor::DJIMotor left_shoot_motor("left_shoot_motor", Motor::DJIMotor::M3508,
    (Motor::DJIMotor::Param) {.id = 0x02,.port = E_CAN2,.mode = Motor::DJIMotor::CURRENT}
);
Motor::DJIMotor right_shoot_motor("right_shoot_motor", Motor::DJIMotor::M3508,
    (Motor::DJIMotor::Param) {.id = 0x01,.port = E_CAN2,.mode = Motor::DJIMotor::CURRENT}
);
Motor::DJIMotor trigger_motor("trigger_motor", Motor::DJIMotor::M3508,
    (Motor::DJIMotor::Param) {.id = 0x01,.port = E_CAN3,.mode = Motor::DJIMotor::CURRENT}
);

Motor::DJIMotor yaw_motor("yaw_motor",Motor::DJIMotor::GM6020,
    {.id = 0x01, .port = E_CAN3, .mode = Motor::DJIMotor::CURRENT});
Motor::DMMotor pit_motor("pit_motor",Motor::DMMotor::J4310,
    {.slave_id = 0x21, .master_id = 0x11, .port = E_CAN1,
        .mode = Motor::DMMotor::MIT, .p_max = 12.5, .v_max = 30, .t_max = 10,
        .kp_max = 500, .kd_max = 5});

Controller::PID yaw_pos(15,0,0,10,0);
Controller::PID yaw_speed(1.5,0.1/1000.0f,0,3,0.2);

Controller::PID fric_speed(10,1,0,16384,3000);
Controller::PID trigger_speed(20,0,0,16384,10000);


auto ins = app_ins_data();
auto rc = bsp_rc_data();

Gimbal::snap robo_snap(&yaw_motor, &pit_motor,&trigger_motor,
    &left_shoot_motor,&right_shoot_motor,ins, PITCH_ZERO_POINT, 0.0f);
Gimbal::Head my_head(&yaw_motor,&pit_motor,&robo_snap);
Gimbal::Shoot my_shoot(&left_shoot_motor, &right_shoot_motor, &trigger_motor,&robo_snap);
Gimbal::vision_core my_vision_core(1000, 60, &robo_snap);

void app_gimbal_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready())
        OS::Task::SleepMilliseconds(10);
    yaw_motor.init();
    yaw_motor.enable();
    pit_motor.init();
    pit_motor.enable();
    PathReference::second_order pitch_path(1000, 30, 1);
    PathReference::second_order yaw_path(1000, 40, 1);
    float target_yaw = 0;
    float target_pitch = 0;
    while(true) {

        robo_snap.snap_update();
        my_head.head_update();
        my_vision_core.tick();
        my_shoot.shoot_tick();

        auto snap = robo_snap.get_snap_pkg();
        auto vision_target = my_vision_core.get_target();

        if(rc->s_l == 1) {
            float raw_pit = ((float)(rc->rc_r[1]))/640.0f;
            target_pitch = pitch_path.update(raw_pit);
            target_yaw += ((float)(rc->rc_r[0]))/640.0f*0.01f;
            target_yaw > PI ? target_yaw -= 2*PI : (target_yaw < -PI ? target_yaw += 2*PI : 0);
            my_head.head_pid_clc(target_yaw,target_pitch);
            my_head.head_output();
        }
        else if(rc->s_l == -1) {
            if(vision_target.target_pitch != 0 || vision_target.target_yaw != 0) {
                target_pitch = pitch_path.update(vision_target.target_pitch);
                target_yaw = vision_target.target_yaw;
            }
            target_yaw < -PI/2? target_yaw = -PI/2:(target_yaw > PI/2? target_yaw = PI/2:0);
            my_head.head_pid_clc(target_yaw,target_pitch);
            my_head.head_output();
        }
        else {
            target_pitch = 0, target_yaw = 0;
            pitch_path.clear();
            my_head.head_relax();
        }


        // if(rc->s_r == 1)
        //     my_shoot.shoot_update(Gimbal::E_FRIC_FAST, Gimbal::E_TRIGGER_FAST);
        // else if(rc->s_r == -1)
        //     my_shoot.shoot_update(Gimbal::E_FRIC_FAST, Gimbal::E_TRIGGER_REST);
        // else
        //     my_shoot.shoot_update(Gimbal::E_FRIC_REST, Gimbal::E_TRIGGER_REST);

        //自瞄测试代码
        if(rc->s_r == -1) {
            my_shoot.shoot_update(Gimbal::E_FRIC_REST, Gimbal::E_TRIGGER_REST);
        }
        else if(rc->s_r == 0) {
            if(vision_target.fire_ctrl_cmd == 1) {
                my_shoot.shoot_update(Gimbal::E_FRIC_FAST, Gimbal::E_TRIGGER_REST);
            }
            else if(vision_target.fire_ctrl_cmd == 2) {
                my_shoot.shoot_update(Gimbal::E_FRIC_FAST, Gimbal::E_TRIGGER_FAST);
            }
            else {
                my_shoot.shoot_update(Gimbal::E_FRIC_REST, Gimbal::E_TRIGGER_REST);
            }
        }
        else if(rc->s_r == 1) {
            my_shoot.shoot_update(Gimbal::E_FRIC_FAST, Gimbal::E_TRIGGER_FAST);
        }


        bsp_uart_printf(E_UART_DEBUG,"%f,%f\r\n",
            (float)vision_target.fire_ctrl_cmd, snap.ins_pit);
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

    my_vision_core.init();

    my_head.head_init(yaw_pos,yaw_speed);
    my_shoot.shoot_param_set(fric_speed, trigger_speed);
}

#endif
