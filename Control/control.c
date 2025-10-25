#include "control.h"

bool start_flag = 0;
bool write_flag = 0;
int16_t control_speed = 500;
void friction_init()
{
    if (!start_flag)
    {

            DJMotorPtr motorp = &DJMotors[0];
			motorp->motorMode = Speed;
motorp = &DJMotors[1];
            motorp->motorMode = Speed;
					control_speed=1000;write_flag=0;
        
    }
    start_flag = 1;
}