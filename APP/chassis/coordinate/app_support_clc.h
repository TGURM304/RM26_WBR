//
// Created by 15082 on 2026/3/22.
//

#ifndef APP_OFF_GROUND_H
#define APP_OFF_GROUND_H
#include "app_snap.h"

namespace Coordinate {
typedef struct {
    float L0_dot2_, theta_dot2_;
    float L0_dot_, theta_dot;
    float theta_, L0_;
    float force_;
    float tor_;
}support_struct;
typedef struct {
    float left_support_, right_support_;
    float support_limit_;
}support_force;
typedef enum {
    E_left,
    E_right
}leg_switch;
class support {
public:
    support() = default;
    support(float force_limit) {
        force_.support_limit_ = force_limit;
    }
    void leg_data_update(robot_snap* snap,observer_struct* observer,
        float left_force, float left_tor, float right_force, float right_tor);
    void support_clc(leg_switch leg);
    support_force get_support(){ return force_;}
private:
    float body_z_dot2_ = 0;
    support_struct left_data_ = {};
    support_struct right_data_ = {};
    support_force force_ = {};
};
}



#endif //APP_OFF_GROUND_H
