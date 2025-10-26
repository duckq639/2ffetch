#include "control.h"

bool start_flag = 0;
bool write_flag = 0;

bool reachout_flag = 0;
bool retract_flag = 0;

float stroke_angle = 0;
float stroke_pos = 0;
int16_t control_speed = 500;
void friction_init()
{
    if (!start_flag)
    {

        DJMotorPtr motorp = &DJMotors[0]; // 摩擦轮初始化
        motorp->motorMode = Speed;
        motorp = &DJMotors[1];
        motorp->motorMode = Speed;

        motorp = &DJMotors[2];
        motorp->motorMode = Position; // 驱动轮初始化

        control_speed = 1000;
        stroke_pos = 200;
        write_flag = 0;
    }
    start_flag = 1;
}
