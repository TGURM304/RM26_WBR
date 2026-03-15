//
// Created by 15082 on 2026/3/8.
//

#ifndef APP_SECOND_ORDER_H
#define APP_SECOND_ORDER_H



namespace PathReference {
class second_order{
public:
    second_order() = default;
    second_order(float update_freq, float omega_n,
        float epsilon)
    : T_(1/update_freq), omega_n_(omega_n), epsilon_(epsilon){
        x1_ = 0;
        x2_ = 0;
    }
    void clear();
    float update(float target);

    void target_set(float current);
    float update_limit(float target,float range);
    void param_set(float update_freq, float omega_n, float epsilon);
private:
    float update_mod(float target);
    float T_{};
    float omega_n_{}; //自然频率，单位为rad/s，数值越大响应越快，但过大可能会引起震荡，通常设置在20~40之间
    float x1_{}, x2_{};//x1_为输出值，x2_为x1_的下一循环积分量
    float last_x1_{}, last_x2_{};
    float process_target_ = 0;//经过处理后的外界输入，在limit中被映射为RR
    float epsilon_ = 1; // 阻尼比，一般都是1，即临界阻尼
};
}



#endif //APP_SECOND_ORDER_H
