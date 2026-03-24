//
// Created by 15082 on 2026/1/19.
//

#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H

//物理常数
#define PI_F32 (3.1415926f)
#define G (9.82f)

//控制数据


//机器人数据，常用宏定义

#define WHEEL_R 0.058
#define RL (0.4386f/2.0f)
#define mess_wheel (0.627f)

//减速箱相关
#define REDUCTION_ORG (3591.0f/187.0f)
#define REDUCTION_NOW (268.0f/17.0f)
#define TORQUE_CONST (0.3f)
#define M3508_LIMIT (20.0f)

#define SGN(x) ((x) > 0 ? 1.0f : -1.0f)


#endif //ROBOT_DATA_H
