//
// Created by 15082 on 2026/1/16.
//

#ifndef APP_MOTOR_PKG_H
#define APP_MOTOR_PKG_H
#include "dev_motor_dji.h"
#include "dev_motor_dm.h"

#include <arm_math_types.h>
namespace Motor_Pkg {
    typedef struct {
        float old_pos, old_speed;
        float pos;
        float speed;
    }motor_status_pkg;
    typedef enum {
        E_forward,
        E_backward
    }E_dir;
    class Joint: Motor::DMMotor {
    public:
        explicit Joint(const char *name, const Model &model,
            const Param &param,E_dir dir, float32_t zero)
            :DMMotor(name,model,param), zero_(zero){
            if(dir == E_forward)
                dir_ = 1.0f;
            else if(dir == E_backward)
                dir_ = -1.0f;
        }
        void pkg_init();
        void set_tor(float tor);
        void rest();
        motor_status_pkg get_status();
    private:
        motor_status_pkg status_ = {0};
        float dir_, zero_;
    };
    class Dynamic: Motor::DJIMotor {
    public:
        explicit Dynamic(const char *name, const Model &model, const Param &param,
            E_dir dir, float reduction, float wheel_R)
        :DJIMotor(name,model,param), reduction_(reduction), wheel_R_(wheel_R){
            if(dir == E_forward)
                dir_ = 1.0f;
            else if(dir == E_backward)
                dir_ = -1.0f;
        }
        void pkg_init();
        void set_tor(float tor);
        void rest();
        void status_clear();
        motor_status_pkg get_status();
    private:
        motor_status_pkg status_ = {0};
        float dir_;
        float reduction_;
        float wheel_R_;
        float encode_ = 0.0f, old_encode_ = 0.0f;
    };
}



#endif //APP_MOTOR_PKG_H
