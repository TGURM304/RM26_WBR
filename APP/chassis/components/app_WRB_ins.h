//
// Created by 15082 on 2026/1/17.
//

#ifndef APP_WRB_INS_H
#define APP_WRB_INS_H
#include "app_ins.h"
#include "SJTU_Matrix/matrix.h"

/*
 * 定义，机体正方向为X轴正方向，向上为Z轴正方向
 * X轴为roll，Z轴为Yaw，Y轴为Pitch
 * 记为：roll，phi，theta
 */
namespace INS{
    typedef struct {
        float body_theta, body_phi, body_roll;
        float body_theta_gry,body_phi_gry,body_roll_gry;
        float body_x_acc,body_y_acc,body_z_acc;
        //机体在出现倾倒的时候会出现万向节死锁的问题
        //导致部分角度出现问题，此时我们引入姿态的目标向量来得到倾倒时的姿态
        float vector_x, vector_y, vector_z;
    }body_posture;
    class app_WRB_ins {
    public:
        app_WRB_ins(const app_ins_data_t *ins):ins_(ins) {
        }
        void set_correct(float correct_rad) {
            correct_rad_ = correct_rad;
        };
        void update();
        const body_posture *get_pos();
    private:
        const app_ins_data_t *ins_;
        body_posture posture_;
        float correct_rad_ = 0;
    };
}




#endif //APP_WRB_INS_H
