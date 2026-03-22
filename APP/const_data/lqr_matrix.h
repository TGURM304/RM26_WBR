//
// Created by 15082 on 2026/1/24.
//

#ifndef LQR_MATRIX_H
#define LQR_MATRIX_H

#define LEG_LEN1  0.171143f
#define LEG_LEN2  0.218108f
#define LEG_LEN3  0.274466f
#define LEG_LEN4  0.321431f
#define LEG_LEN5  0.340217f

extern const float leg_len_arr[5];
extern const float K17[4][10];
extern const float K22[4][10];
extern const float K27[4][10];
extern const float K32[4][10];
extern const float K34[4][10];

extern const float (*K_ary[])[10];

extern const float K_Fit_Coefficients[40][6] ;

#endif //LQR_MATRIX_H
