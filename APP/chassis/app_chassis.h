//
// Created by fish on 2024/11/16.
//

#pragma once

#include "app_conf.h"
#include "app_WRB_ins.h"
#include "app_ins.h"
#include "app_sys.h"
#include "bsp_uart.h"
#include "sys_task.h"
#include "app_observer.h"
#include "app_vmc.h"
#include "robot_data.h"

#ifdef __cplusplus
extern "C" {
#endif

void app_chassis_init();
void app_chassis_task(void *argument);

#ifdef __cplusplus
}
#endif