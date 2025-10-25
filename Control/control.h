#pragma once
#include "dj_motor.h"

extern bool start_flag;
extern bool write_flag;
extern int16_t control_speed;

void friction_init();
/* 写控制任务原型（在 freertos 中实现） */
void WriteControlTask(void *argument);
