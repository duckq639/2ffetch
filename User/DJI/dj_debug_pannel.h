#ifndef DJ_DEBUG_PANNEL_H
#define DJ_DEBUG_PANNEL_H
#include "dj_motor.h"
#include "pid.h"
#include "stdbool.h"
typedef struct
{
    uint8_t DJ_ID;
    DJMotorType DJ_MType;
    DJMotorMode DJ_MMode;

    PIDtype posPID;
    PIDtype rpmPID;

    float DJ_SetAngle;
    int16_t DJ_SetSpeed;

    float DJ_ReadAngle;
    int16_t DJ_ReadSpeed;

    bool DJ_SetZero;

} DJ_DEBUG_PANNEL;

extern DJ_DEBUG_PANNEL DJ_DEBUG;
extern uint8_t DJ_ID_Set;
void DJ_Debug_Monitor();
#endif
