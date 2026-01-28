//
// Created by 15082 on 2026/1/27.
//

#ifndef APP_CONTROL_PIPELINE_H
#define APP_CONTROL_PIPELINE_H

#include "app_LQR.h"
#include "app_leg_ctrl.h"
#include "app_relay.h"
#include "app_vmc.h"
#include "ctrl_forward_feed.h"
#include "robot_data.h"


namespace Pipeline {
typedef struct {
    float32_t tor1, tor2, tor3, tor4;//电机id对应扭矩
    float32_t dynamic_right, dynamic_left;//动力轮扭矩
}final_output;
typedef struct {
    float32_t delta_S;
    float32_t delta_ver_left;
    float32_t delta_ver_right;
    uint16_t slip_cnt;
} ver_ctrl_switch;
typedef struct {
    float32_t T_left_W, T_right_W;
    float32_t T_left_B, T_right_B;
}lqr_out;
typedef enum {
    E_LQR_STATIC,
    E_LQR_DYNAMIC,
    E_LEG_PID_DISABLE,
    E_LEG_PID_ENABLE,
    E_SLIP_ENABLE,
    E_SLIP_DISABLE,
    E_LQR_JUMP,
    E_CHASSIS_SAFE,
    E_CHASSIS_USING
}ctrl_cmd;
#define DELTA_VER_EDGE (0.5f)
#define DELTA_S_EDGE (0.3f)
#define SLIP_CNT_EDGE_LOW (50)
#define SLIP_CNT_EDGE_HIGH (100)
#define FORWARD_FEED (50.f)
#define ABS(data) ((data)>0?(data):-(data))

#define CHASSIS_REST (0x01)
#define CHASSIS_STATIC_LQR (0x01<<2)
#define CHASSIS_DYNAMIC_LQR (0x01<<3)
#define CHASSIS_JUMP (0x01<<4)
#define LEG_LEN_ENABLE (0x01<<5)
#define SLIP_ENABLE (0x01<<6)
#define CHASSIS_SAFE (0x01<<7)
class control_pipeline {
public:
    control_pipeline() = default;
    control_pipeline( Relay::message_adapter* adapter,  VMC::app_vmc* vmc,
         LQR::LQR_controller *lqr,  LegController::app_leg_ctrl *leg)
    :adapter_(adapter), vmc_(vmc), lqr_(lqr), leg_controller_(leg) {
    };
    void observer_update();
    void data_clear();
    void rest();
    //LQR控制主函数，输入后更新到对应的LQR控制器中
    void lqr_control(float32_t target_ver, float32_t phi_gry);
    void lqr_get();
    void leg_len_control(float32_t left_len, float32_t right_len);
    void vmc_pkg_update();
    void vmc_clc();
    void motor_tor_update();
    void set_state(ctrl_cmd lqr_cmd, ctrl_cmd leg_pid_cmd,
        ctrl_cmd slip_cmd, ctrl_cmd safe_cmd);
    final_output get_motor_tor() {return output_;}
    lqr_out lqr_out_tor_ = {};//LQR输出扭矩

private:
    //组件指针
    Relay::message_adapter* adapter_ = nullptr;
    VMC::app_vmc* vmc_ = nullptr;
    LQR::LQR_controller* lqr_ = nullptr;
    LegController::app_leg_ctrl* leg_controller_ = nullptr;
    //数据缓存
    //系统观测器得到数据
    Relay::relay_lqr lqr_data_ = {};
    Relay::relay_leg left_leg_ = {};
    Relay::relay_leg right_leg_ = {};
    //计算的中间数据
    Relay::relay_lqr lqr_target_ = {};//LQR目标状态
    float32_t lqr_delta_state_[10] = {0};//LQR状态差值
    //控制器输出数据
    VMC::ctrl_pkg ctrl_pkg_right_ = {};//右腿vmc控制包
    VMC::ctrl_pkg ctrl_pkg_left_ = {};//左腿vmc控制包
    VMC::motor_tor vmc_tor_ = {};//vmc计算得到的电机扭矩
    LegController::leg_output leg_pid_output_ = {};//腿部pid输出，用于腿长控制
    final_output output_ = {};//最后合成出来的电机输出扭矩
    //控制状态
    uint32_t control_state_ = CHASSIS_REST;
    ver_ctrl_switch ver_switch_ = {};

};

} // Pipeline

#endif //APP_CONTROL_PIPELINE_H
