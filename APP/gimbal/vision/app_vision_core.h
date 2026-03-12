//
// Created by 15082 on 2026/3/10.
//

#ifndef APP_VISION_CORE_H
#define APP_VISION_CORE_H
#include "app_gimbal_snap.h"
#include "app_snap.h"
#include "app_vision.h"


namespace Gimbal {

typedef struct {
    float target_yaw, target_pitch;
    uint8_t fire_ctrl_cmd;
}vision_pkg;
    class vision_core {
    public:
        vision_core() = default;
        vision_core(float ex_fre, float send_fre,Gimbal::snap *snap_ptr)
        : ex_fre_(ex_fre), send_fre_(send_fre),snap_ptr_(snap_ptr){
            target_cnt = ex_fre_ / send_fre_;
            current_cnt = 0;
            vd = vision::recv();
        };
        void init();
        void tick();
        vision_pkg get_target();
        Gimbal::snap *snap_ptr_ = nullptr;
    private:
        const app_ins_data_t *ins_ = nullptr;
        vision::RecvPacket* vd = nullptr;
        vision_pkg pkg_{};
        float ex_fre_ = 0, send_fre_ = 0;//调用频率和发送频率
        float target_cnt{}, current_cnt{};
    };
}




#endif //APP_VISION_CORE_H
