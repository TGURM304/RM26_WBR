//
// Created by fish on 2024/11/16.
//

#pragma once

#include "app_conf.h"
#include "dev_motor_dji.h"
#include "app_motor.h"
#include "ctrl_motor_base_pid.h"
#include "app_ins.h"
#include "app_sys.h"
#include "bsp_rc.h"
#include "sys_task.h"
#ifdef __cplusplus
extern "C" {
#endif

void app_gimbal_init();
void app_gimbal_task(void *args);

#ifdef __cplusplus
}
#endif