#include "dj_debug_pannel.h"
#include "string.h"

DJ_DEBUG_PANNEL DJ_DEBUG = {0};
uint8_t DJ_ID_Set = 0;

static DJMotorPtr DJ_Debug_Motor = NULL;

void DJ_Debug_Monitor()
{
    static uint8_t last_ID_Set = 0;
    static DJ_DEBUG_PANNEL last_debug = {0}; // 上次调试结构体快照

    // 电机切换检测（整体同步）
    if (DJ_ID_Set != last_ID_Set && DJ_ID_Set <= 8)
    {
        last_ID_Set = DJ_ID_Set;
        if (DJ_ID_Set == 0)
        {
            DJ_Debug_Motor = NULL;
            memset(&DJ_DEBUG, 0, sizeof(DJ_DEBUG_PANNEL));
            return;
        }
        DJ_Debug_Motor = &DJMotors[IDlist[DJ_ID_Set]];
        if (DJ_Debug_Motor) // 切换电机时首先进行参数覆写
        {
            DJ_DEBUG.DJ_MMode = DJ_Debug_Motor->motorMode;
            DJ_DEBUG.posPID = DJ_Debug_Motor->posPID;
            DJ_DEBUG.rpmPID = DJ_Debug_Motor->rpmPID;
            DJ_DEBUG.DJ_SetAngle = DJ_Debug_Motor->valueSet.angle;
            DJ_DEBUG.DJ_SetSpeed = DJ_Debug_Motor->valueSet.speed;
            DJ_DEBUG.DJ_ID = DJ_Debug_Motor->param.StdID;
            DJ_DEBUG.DJ_SetZero = DJ_Debug_Motor->status.setZeroFlag;

            memcpy(&last_debug, &DJ_DEBUG, sizeof(DJ_DEBUG_PANNEL)); // 同步快照
        }
    }

    if (!DJ_Debug_Motor)
        return;

    // 分项检测 + 写入 + 局部快照同步

    if (DJ_DEBUG.DJ_MMode != last_debug.DJ_MMode) // 如果调试面板上的电机模式与上次快照不同
    {
        DJ_Debug_Motor->motorMode = DJ_DEBUG.DJ_MMode; // 将调试面板的新模式写入当前选中电机
        last_debug.DJ_MMode = DJ_DEBUG.DJ_MMode;       // 同步快照
    }

    if (memcmp(&DJ_DEBUG.posPID, &last_debug.posPID, sizeof(PIDtype)) != 0) // 比较位置 PID 结构是否发生变化（按内存字节比较）
    {
        DJ_Debug_Motor->posPID = DJ_DEBUG.posPID; // 若变化则把新的 posPID 复制到电机对象
        last_debug.posPID = DJ_DEBUG.posPID;      // 同步快照
    }

    if (memcmp(&DJ_DEBUG.rpmPID, &last_debug.rpmPID, sizeof(PIDtype)) != 0) // 比较转速 PID 结构是否发生变化
    {
        DJ_Debug_Motor->rpmPID = DJ_DEBUG.rpmPID; // 若变化则把新的 rpmPID 复制到电机对象
        last_debug.rpmPID = DJ_DEBUG.rpmPID;      // 同步快照
    }

    if (DJ_DEBUG.DJ_SetAngle != last_debug.DJ_SetAngle) // 检查角度设定是否改变
    {
        DJ_Debug_Motor->valueSet.angle = DJ_DEBUG.DJ_SetAngle; // 写入电机目标角度
        last_debug.DJ_SetAngle = DJ_DEBUG.DJ_SetAngle;         // 同步快照
    }

    if (DJ_DEBUG.DJ_SetSpeed != last_debug.DJ_SetSpeed) // 检查速度设定是否改变
    {
        DJ_Debug_Motor->valueSet.speed = DJ_DEBUG.DJ_SetSpeed; // 写入电机目标速度
        last_debug.DJ_SetSpeed = DJ_DEBUG.DJ_SetSpeed;         // 同步快照
    }

    if (DJ_DEBUG.DJ_SetZero && !last_debug.DJ_SetZero) // 边沿检测：只有从未触发到触发时才执行一次置零操作
    {
        DJ_Debug_Motor->status.setZeroFlag = true; // 设置电机的置零标志，实际归零逻辑由电机任务处理
        DJ_DEBUG.DJ_SetZero = false;               // 清除调试面板上的触发位（防止重复触发）
        DJ_DEBUG.DJ_SetAngle = 0;
        last_debug.DJ_SetZero = false; // 同步快照
    }

    // 寻零模式下状态读取
    if (DJ_DEBUG.DJ_MMode == Zero && DJ_Debug_Motor->motorMode != Zero) // 寻零模式控制逻辑
    {
        DJ_DEBUG.DJ_MMode = DJ_Debug_Motor->motorMode;
        last_debug.DJ_MMode = DJ_DEBUG.DJ_MMode;
    }

    // 实时更新显示
    DJ_DEBUG.DJ_ID = DJ_Debug_Motor->param.StdID;
    DJ_DEBUG.DJ_ReadAngle = DJ_Debug_Motor->valueNow.angle;
    DJ_DEBUG.DJ_ReadSpeed = DJ_Debug_Motor->valueNow.speed;
}