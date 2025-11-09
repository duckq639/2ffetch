#pragma once
#include "dj_motor.h"

extern bool start_flag;
extern bool write_flag;

extern bool reachout_flag;
extern bool retract_flag;
extern bool action_falg;
extern float stroke_pos;

static float const_ = 1;

extern int16_t control_speed;

void friction_init();
static inline float pos2angle(float pos)
{
    float angle;
    angle = pos / const_;
    return angle;
}
/* 写控制任务原型（在 freertos 中实现） */
void WriteControlTask(void *argument);
void ActionTask(void *argument);