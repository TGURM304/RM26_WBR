//
// Created by fish on 2024/11/17.
//

#include "app_gimbal.h"

#include "app_cmd.h"
#include "app_gimbal_coordinate.h"
#include "app_shoot_controller.h"
#include "app_head_controller.h"
#include "app_keyboard.h"
#include "bsp_uart.h"
#include "dev_motor_dm.h"
#include "app_second_order.h"
#include "app_vision_core.h"
#include "msg.h"
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
                              Motor::DJIMotor::M2006,
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
Gimbal::gimbal_coordinate coordinate(&my_shoot, &my_head, &my_vision_core);

//板间通讯
IBC::ibc_gimbal gimbal_send        = {};
IBC::ibc_chassis_data chassis_data = {};
// app_msg_can_receiver<IBC::ibc_chassis> chassis_receiver(E_CAN3, CHASSIS_ID);

msg::can_sender<IBC::ibc_gimbal, E_CAN3, GIMBAL_ID> sender(2);
msg::can_receiver<IBC::ibc_chassis, E_CAN3, CHASSIS_ID> chassis_test;


int16_t send_cnt = 0;
void app_gimbal_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready())
        OS::Task::SleepMilliseconds(10);
    yaw_motor.init();
    yaw_motor.enable();
    pit_motor.init();
    pit_motor.enable();

    robomaster::image::init(E_UART_7);
    auto image_rc = robomaster::image::rc::data();
    Gimbal::keyboard my_keyboard(image_rc);
    // chassis_receiver.init();

    sender.init();
    chassis_test.init();

    while(true) {
        send_cnt++;
        if(send_cnt >= 5) {
            send_cnt = 0;
            auto cmd = coordinate.ctrl_.get_cmd();
            //获取底盘位姿与云台的相对位姿，然后计算得到底盘的跟随目标位姿
            volatile float motor_deg;
            volatile int32_t delta_encoder = yaw_motor.status.angle - 4168;
            if(delta_encoder > 8192 / 2) delta_encoder -= 8192;
            else if(delta_encoder < -8192 / 2) delta_encoder += 8192;
            motor_deg              = (float)delta_encoder * PI_F32 / (8192/2);
            volatile float body_target_yaw  = motor_deg + chassis_data.body_phi;
            gimbal_send.target_yaw = IBC::float32_to_uint16(body_target_yaw, PI_F32, -PI_F32);
            gimbal_send.height     = IBC::float32_to_uint16(cmd->target_height, ZH_HEIGHT_MAX, ZH_HEIGHT_MIN);
            gimbal_send.vx         = IBC::float32_to_uint16(cmd->vx, ZH_V_MAX, ZH_V_MIN);
            gimbal_send.vy         = IBC::float32_to_uint16(cmd->vy, ZH_V_MAX, ZH_V_MIN);
            gimbal_send.switch_cmd = coordinate.ctrl_.get_flag()->switch_cmd_;

            UNUSED(delta_encoder);
            UNUSED(motor_deg);
            UNUSED(body_target_yaw);

            *sender() = gimbal_send;
            sender.send();

            auto ibc                  = chassis_test();
            chassis_data.vector_x     = IBC::uint16_to_float32(ibc->vector_x, 1, -1);
            chassis_data.vector_y     = IBC::uint16_to_float32(ibc->vector_y, 1, -1);
            chassis_data.vector_z     = IBC::uint16_to_float32(ibc->vector_z, 1, -1);
            chassis_data.body_phi     = IBC::uint16_to_float32(ibc->body_phi, PI_F32, -PI_F32);
            chassis_data.chassis_cmd_ = ibc->chassis_cmd_;
        }

        my_keyboard.update();
        auto temp = my_keyboard.get_pkg();

        if(pit_motor.status.err != 3) {
            pit_motor.reset();
            pit_motor.enable();
        }

        robo_snap.snap_update();
        coordinate.update_keyboard(temp);
        coordinate.tick();
        app_msg_vofa_send(E_UART_1, yaw_motor.status.angle);
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
