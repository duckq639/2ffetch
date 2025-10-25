#include "pid.h"
#include <math.h>
#define ABS(x) ((x) > 0 ? (x) : -(x)) // 绝对值
/**
 * @brief
 *
 * @param pid
 * @param kp
 * @param ki
 * @param kd
 * @param set
 * @return int32_t 返回pid输出量
 */
void PID_Init(PIDtype *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setval = 0;
    pid->curval = 0;
    pid->err = 0;
    pid->err_prev = 0;
    pid->err_last = 0;
    pid->delta = 0;
    pid->delta_prev = 0;
}
int32_t PID_Calculate_Delta(PIDtype *pid)
{

    pid->err = pid->setval - pid->curval;
    if (ABS(pid->err) < PID_ERR_DEADZONE)
        pid->err = 0;
    if (ABS(pid->err) <= PID_ERR_REDUCE_THRESHOLD) // 误差小于xx减弱
    {
        pid->delta = pid->kp * (pid->err - pid->err_last) + pid->ki * PID_OUTPUT_REDUCE * pid->err + pid->kd * PID_OUTPUT_REDUCE * (pid->err - 2 * pid->err_last + pid->err_prev);
    }
    else
    {
        pid->delta = pid->kp * (pid->err - pid->err_last) + pid->ki * pid->err + pid->kd * (pid->err - 2 * pid->err_last + pid->err_prev);
    }
    pid->err_prev = pid->err_last;
    pid->err_last = pid->err; // 顺序不能乱
    pid->delta = roundf(pid->delta_prev * 0.2f + pid->delta * 0.8f);
    pid->delta_prev = pid->delta;
    return (int32_t)pid->delta;
}
