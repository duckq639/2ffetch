#pragma once
#include "stm32f405xx.h"
#define PID_ERR_DEADZONE 15    // 误差死区
#define PID_DELTA_MIN_OUTPUT 5 // 输出最小值
#define PID_ERR_REDUCE_THRESHOLD 350
#define PID_OUTPUT_REDUCE 0.81f // 低幅误差减弱系数

typedef struct
{
    float kp;
    float ki;
    float kd;
    volatile int32_t setval;
    volatile int32_t curval;
    volatile int32_t err;
    volatile int32_t err_last;
    volatile int32_t err_prev;
    volatile int32_t delta;
    volatile int32_t delta_prev;
} PIDtype;
void PID_Init(PIDtype *pid, float kp, float ki, float kd); // 初始化
int32_t PID_Calculate_Delta(PIDtype *pid);                 // PID计算
