#include "dj_motor.h"
#include "math.h"
/*=============================================类型定义=======================================================*/
DJMotor DJMotors[USE_DJ_NUM];
CAN_Frame DJCAN_TxFrame = {
    .CAN_FState = CAN_FSTATE_IDLE,
    .CAN_RxHeader = {0},
    .CAN_TxHeader = {.DLC = 8},
    .hcan = &DJ_CAN_CHANNEL,
    .Data = {0}};
CAN_Frame DJCAN_RxFrame = {
    .CAN_FState = CAN_FSTATE_IDLE,
    .CAN_TxHeader = {0},
    .CAN_RxHeader = {.DLC = 8},
    .hcan = &DJ_CAN_CHANNEL,
    .Data = {0}};

#define INVALID_ID (-1)
int8_t IDlist[9] = {INVALID_ID, INVALID_ID, INVALID_ID, INVALID_ID, INVALID_ID, INVALID_ID, INVALID_ID, INVALID_ID, INVALID_ID};
/*=============================================函数定义=======================================================*/
/*----------------------------------工具函数声明-----------------------------------*/
void DJMotor_Angle_Calculate(DJMotorPtr motorp);
void DJMotor_UpdateDir(DJMotorPtr motorp);
static inline float Angle_ToPulse(DJMotorPtr motorp, float angle);
static inline float Pulse_ToAngle(DJMotorPtr motorp, int32_t pulse); // 计算时保留精度
void Set_Zero(DJMotorPtr motorp);
void Set_Status(DJMotorPtr motorp);
static inline void EncodeS16Data(volatile int16_t *value, uint8_t *buffer);
static inline int16_t DecodeS16Data(uint8_t *src);
/*--------------------------------初始化函数------------------------------------*/
void DJMotor_Init(DJMotorPtr DJMotorsp, uint16_t ID, DJMotorType mtype)
{
    memset(DJMotorsp, 0, sizeof(*DJMotorsp));
    switch (mtype)
    {
    case DJM2006:
        DJMotorsp->param.StdID = ID; // 可供更改
        DJMotorsp->param.reductionRatio = M2006_RATIO;
        DJMotorsp->param.gearRatio = 1; // 改->机构减速⽐
        DJMotorsp->param.pulsePerRound = 8191;
        PID_Init(&DJMotorsp->posPID, 1.35, 0.32, 0);
        PID_Init(&DJMotorsp->rpmPID, 8, 0.5, 0.02);
        DJMotorsp->motorType = DJM2006;
        break;
    case DJM3508:
        DJMotorsp->param.StdID = ID; // 可供更改
        DJMotorsp->param.reductionRatio = M3508_RATIO;
        DJMotorsp->param.gearRatio = 1;
        DJMotorsp->param.pulsePerRound = 8191;
        DJMotorsp->param.currentLimit = 10000;
        PID_Init(&DJMotorsp->posPID, 2, 0.08, 0);
        PID_Init(&DJMotorsp->rpmPID, 8, 0.5, 0);
        DJMotorsp->motorType = DJM3508;
        break;
    default:
        break;
    }
    DJMotorsp->conter.lastRxTime = 0;
    DJMotorsp->conter.zeroCnt = 0;
    DJMotorsp->conter.deadZoneCnt = 0;
    DJMotorsp->conter.pulseLock = 0;
    DJMotorsp->conter.maxDistance = 0;
    DJMotorsp->conter.pawLockCnt = 0;
    DJMotorsp->conter.dirCnt = 0;

    DJMotorsp->limit.isPosAngleLimit = false; // 位置模式
    DJMotorsp->limit.posRPMFlag = true;
    DJMotorsp->limit.posRPMLimit = 4000;
    DJMotorsp->limit.maxAngle = 300;
    DJMotorsp->limit.minAngle = -300;

    DJMotorsp->limit.isRPMLimit = true;     //  速度模式
    DJMotorsp->limit.isCurrentLimit = true; //  是否电流达到限制
    DJMotorsp->limit.isStuck = false;       //  是否堵转
    DJMotorsp->limit.zeroSpeed = 1000;
    DJMotorsp->limit.zeroCurrent = 3000;
    DJMotorsp->limit.deadZoneEnable = true; // 启动角度死区检测,消除微小抖动

    DJMotorsp->status.isArrived = false;
    DJMotorsp->status.isOvertime = false;
    DJMotorsp->status.setZeroFlag = true;
    DJMotorsp->status.isStuck = false;
    DJMotorsp->status.isZeroed = false;
    DJMotorsp->status.findZeroDir = 0; // 反向寻零
    DJMotorsp->status.dirFlag = 1;

    DJMotorsp->error.releaseWhenStuck = false;
    DJMotorsp->error.stuckDetected = true;
    DJMotorsp->error.timeoutDetected = true;
    DJMotorsp->error.timeoutCount = 0;
    DJMotorsp->error.stuckCount = 0;

    memset(&DJMotorsp->valueSet, 0, sizeof(DJMotorsp->valueSet));
    memset(&DJMotorsp->valueNow, 0, sizeof(DJMotorsp->valueNow));
    memset(&DJMotorsp->valuePre, 0, sizeof(DJMotorsp->valuePre));

    DJMotorsp->begin = true;
    DJMotorsp->motorMode = Disable;
    uint16_t arrid = (uint16_t)(DJMotorsp - DJMotors);
    IDlist[ID] = arrid;
}
/*--------------------------------控制函数------------------------------------*/

void DJMotor_Lock_Position(DJMotorPtr motorp)
{
    motorp->posPID.setval = motorp->conter.pulseLock;
    motorp->posPID.curval = motorp->valueNow.pulsetotal;

    motorp->rpmPID.setval = PID_Calculate_Delta(&(motorp->posPID));
    motorp->rpmPID.curval = motorp->valueNow.speed;

    motorp->valueSet.current += PID_Calculate_Delta(&(motorp->rpmPID));

    PEAK(motorp->valueSet.current, 6000);
    EncodeS16Data(&motorp->valueSet.current, &DJCAN_TxFrame.Data[(motorp->param.StdID - 1) * 2]);
}

void DJMotor_Position_Mode(DJMotorPtr motorp) // 位置模式
{
    motorp->valueSet.pulsetotal = Angle_ToPulse(motorp, motorp->valueSet.angle);
    if (motorp->limit.isPosAngleLimit == true) // 启动运动角度限制
    {
        int maxpulse = Angle_ToPulse(motorp, motorp->limit.maxAngle);
        int minpulse = Angle_ToPulse(motorp, motorp->limit.minAngle);
        MIN(motorp->valueSet.pulsetotal, maxpulse);
        MAX(motorp->valueSet.pulsetotal, minpulse);
        // 运动方向补偿,小距离移动不启用补偿
        motorp->posPID.setval = motorp->valueSet.pulsetotal + (POSETOIN_DISTANCE_THRESHOLD - POSETOIN_BIAS_COMP) * signum(motorp->status.dirFlag);
    }
    else
    {
        // 运动方向补偿,小距离移动不启用补偿
        motorp->posPID.setval = motorp->valueSet.pulsetotal + (POSETOIN_DISTANCE_THRESHOLD - POSETOIN_BIAS_COMP) * signum(motorp->status.dirFlag);
    }
    motorp->posPID.curval = motorp->valueNow.pulsetotal;
    motorp->rpmPID.setval = PID_Calculate_Delta(&motorp->posPID);
    motorp->rpmPID.curval = motorp->valueNow.speed;

    if (motorp->limit.posRPMFlag == true) // 峰值限制
    {
        PEAK(motorp->rpmPID.setval, motorp->limit.posRPMLimit);
    }
    motorp->valueSet.current += PID_Calculate_Delta(&motorp->rpmPID);
    DJMotor_UpdateDir(motorp);
    EncodeS16Data(&motorp->valueSet.current, &DJCAN_TxFrame.Data[(motorp->param.StdID - 1) * 2]);
}

void DJMotor_Speed_Mode(DJMotorPtr motorp) // 速度模式
{
    motorp->rpmPID.setval = motorp->valueSet.speed;
    motorp->rpmPID.curval = motorp->valueNow.speed;
    motorp->valueSet.current += PID_Calculate_Delta(&motorp->rpmPID);
    EncodeS16Data(&motorp->valueSet.current, &DJCAN_TxFrame.Data[(motorp->param.StdID - 1) * 2]);
    motorp->valueSet.angle = motorp->valueNow.angle; // 防止速度切位置时疯转
}

void DJMotor_Zero_Mode(DJMotorPtr motorp)
{
    motorp->rpmPID.setval = motorp->limit.zeroSpeed;
    motorp->rpmPID.curval = motorp->valueNow.speed;
    if (0 == motorp->status.findZeroDir) // 寻零方向逻辑
    {
        motorp->rpmPID.setval = -motorp->limit.zeroSpeed;
    }
    motorp->valueSet.current += PID_Calculate_Delta(&motorp->rpmPID);
    PEAK(motorp->valueSet.current, motorp->limit.zeroCurrent); // 寻零速度限幅

    EncodeS16Data(&motorp->valueSet.current, &DJCAN_TxFrame.Data[(motorp->param.StdID - 1) * 2]);

    if (ABS(motorp->valueNow.pulsedistance) < ZERO_DISTANCE_THRESHOLD)
    {
        if (motorp->conter.zeroCnt++ > 25)
        {
            motorp->conter.zeroCnt = 0;
            motorp->status.isArrived = true;
            Set_Zero(motorp);
            motorp->motorMode = Enable;
        }
    }
}
void DJMotor_Disable(DJMotorPtr motorp)
{
    motorp->valueSet.current = 0;
    EncodeS16Data(&motorp->valueSet.current, &DJCAN_TxFrame.Data[(motorp->param.StdID - 1) * 2]);
}
/*--------------------------------命令函数------------------------------------*/
void DJMotor_Send_Cmd(uint16_t DJCAN_ID)
{
    uint32_t mailbox = 0;

    if (HAL_CAN_GetTxMailboxesFreeLevel(DJCAN_TxFrame.hcan) == 0)
    {
        // 邮箱满，直接返回或做等待处理
        return;
    }
    if (4 == DJCAN_ID)
    {
        DJCAN_TxFrame.CAN_TxHeader.StdId = 0x200;
        HAL_CAN_AddTxMessage(DJCAN_TxFrame.hcan, &DJCAN_TxFrame.CAN_TxHeader, DJCAN_TxFrame.Data, &mailbox);
    }
    else if (8 == DJCAN_ID)
    {
        DJCAN_TxFrame.CAN_TxHeader.StdId = 0x1FF;
        HAL_CAN_AddTxMessage(DJCAN_TxFrame.hcan, &DJCAN_TxFrame.CAN_TxHeader, DJCAN_TxFrame.Data, &mailbox);
    }
}

void DJMotor_Read_Cmd(CAN_RxHeaderTypeDef *rxhdr) // 放置在can中断回调函数中,在此函数之前先getmessage存入头
{
    if (rxhdr->StdId >= 0x201 && rxhdr->StdId <= 0x208)
    {
        uint8_t ID = rxhdr->StdId - 0x200;
        DJMotors[IDlist[ID]].valuePre = DJMotors[IDlist[ID]].valueNow;
        DJMotors[IDlist[ID]].valueNow.pulseread = DecodeS16Data(&DJCAN_RxFrame.Data[0]);
        DJMotors[IDlist[ID]].valueNow.speed = DecodeS16Data(&DJCAN_RxFrame.Data[2]);
        DJMotors[IDlist[ID]].valueNow.current = DecodeS16Data(&DJCAN_RxFrame.Data[4]);

        if (DJMotors[IDlist[ID]].param.reductionRatio == M3508_RATIO)
        {
            DJMotors[IDlist[ID]].valueNow.speed = (float)DJMotors[IDlist[ID]].valueNow.speed; // 原来左边的是realspeed,但是不知道有啥用
            DJMotors[IDlist[ID]].valueNow.temperature = DJCAN_RxFrame.Data[6];
            DJMotors[IDlist[ID]].valueNow.currentA = (float)DJMotors[IDlist[ID]].valueNow.current * 0.0012207f;
        }
        else
        {
            DJMotors[IDlist[ID]].valueNow.currentA = (float)DJMotors[IDlist[ID]].valueNow.current * 0.001f;
        }
        DJMotor_Angle_Calculate(&DJMotors[IDlist[ID]]);
        DJMotors[IDlist[ID]].conter.lastRxTime = 0;
    }
}
// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
//     HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &DJCAN_RxFrame.CAN_RxHeader, DJCAN_RxFrame.Data);
//     if (hcan->Instance == DJCAN_RxFrame.CAN_X)
//     {
//         DJMotor_Read_Cmd(&DJCAN_RxFrame.CAN_RxHeader);
//     }
// }
/*--------------------------------集和封装函数------------------------------------*/
void DJMotor_Func()
{
    static DJMotorPtr motorp;
    for (uint16_t id = 1; id <= 8; id++)
    {
        if (INVALID_ID == IDlist[id])
        {
            DJMotor_Send_Cmd(id);
            continue;
        }
        else
        {
            if (motorp->status.setZeroFlag == true)
            {
                Set_Zero(motorp);
            }
            motorp = &DJMotors[IDlist[id]];
            if (motorp->begin)
            {
                switch (motorp->motorMode)
                {
                case Disable:
                    DJMotor_Disable(motorp);
                    break;
                case Enable:
                    DJMotor_Lock_Position(motorp);
                    break;
                case Speed:
                    DJMotor_Speed_Mode(motorp);
                    break;
                case Position:
                    DJMotor_Position_Mode(motorp);
                    break;
                case Zero:
                    DJMotor_Zero_Mode(motorp);
                    break;
                default:
                    break;
                }
                DJMotor_Send_Cmd(id);
            }
            else
            {
                continue;
            }
        }
    }
}
/*--------------------------------工具函数------------------------------------*/
void DJMotor_Angle_Calculate(DJMotorPtr motorp)
{
    motorp->valueNow.pulsedistance = motorp->valueNow.pulseread - motorp->valuePre.pulseread;

    if (ABS(motorp->valueNow.pulsedistance) > 4096)
    {
        motorp->valueNow.pulsedistance = motorp->valueNow.pulsedistance - signum(motorp->valueNow.pulsedistance) * motorp->param.pulsePerRound;
    }
    motorp->valueNow.pulsetotal += motorp->valueNow.pulsedistance;
    motorp->valueNow.angle = Pulse_ToAngle(motorp, motorp->valueNow.pulsetotal);
    if (motorp->begin && motorp->motorMode != Enable) // lock值更新,使能状态除外,确保切换到使能模式不会疯转
    {
        motorp->conter.pulseLock = motorp->valueNow.pulsetotal;
    }
    if (motorp->status.setZeroFlag) // 置零标志位
    {
        Set_Zero(motorp);
    }
    if (ABS(motorp->valueNow.pulsedistance) < POSETOIN_DISTANCE_THRESHOLD &&
        ABS(motorp->posPID.err) < POSETOIN_DISTANCE_THRESHOLD &&
        motorp->motorMode == Position && motorp->motorMode == Enable) // 强制消除抖动,会出现一定脉冲读数滑移
    {
        // if (motorp->conter.deadZoneCnt++ > 3)//计数被判定不通过,增加计数只会更抖
        // {
        // motorp->conter.deadZoneCnt = 0;
        // motorp->valueSet.angle = motorp->valueNow.angle;
        // motorp->valueSet.pulsetotal = motorp->valueNow.pulsetotal;
        float scale = motorp->valueNow.pulsedistance / (float)POSETOIN_DISTANCE_THRESHOLD; // 0~1之间
        motorp->valueSet.current *= (0.1f + 0.7f * ABS(scale));                            // 距离越近，输出越弱
        // }
        // motorp->valueNow.pulsedistance = 0;
    }
}
void DJMotor_UpdateDir(DJMotorPtr motorp)
{
    if (ABS(motorp->valueNow.pulsedistance) > POSETOIN_DISTANCE_THRESHOLD / 15)
    {
        motorp->conter.dirCnt += signum(motorp->valueNow.pulsedistance);
    }
    else
    {
        if (motorp->conter.dirCnt > 0)
            motorp->conter.dirCnt--;
        else if (motorp->conter.dirCnt < 0)
            motorp->conter.dirCnt++;
    }
    if (ABS(motorp->conter.dirCnt) >= 150)
    {
        motorp->status.dirFlag = signum(motorp->conter.dirCnt);
        motorp->conter.dirCnt = motorp->status.dirFlag * 20;
    }
    else if (ABS(motorp->conter.dirCnt) <= 5)
    {
        motorp->status.dirFlag = 0;
    }
}
static inline float Angle_ToPulse(DJMotorPtr motorp, float angle)
{
    return roundf(angle / 360.0f /*0.0027777f*/ * motorp->param.reductionRatio * motorp->param.gearRatio * motorp->param.pulsePerRound);
}
static inline float Pulse_ToAngle(DJMotorPtr motorp, int32_t pulse)
{
    return pulse * 360.0f / (motorp->param.pulsePerRound * motorp->param.reductionRatio * motorp->param.gearRatio);
}
void Set_Zero(DJMotorPtr motorp)
{
    motorp->status.setZeroFlag = false;
    motorp->valueNow.angle = 0;
    motorp->valueNow.pulsetotal = 0;
    motorp->valueSet.angle = 0;
    motorp->valueSet.pulsetotal = 0;
    motorp->conter.pulseLock = 0;
}
void Set_Status(DJMotorPtr motorp)
{
    motorp->status.isArrived = false;
    motorp->status.isOvertime = false;
    motorp->status.setZeroFlag = false;
    motorp->status.isStuck = false;
    motorp->status.isZeroed = false;
}
// 编码时（int16_t -> data）
static inline void EncodeS16Data(volatile int16_t *value, uint8_t *buffer)
{
    buffer[0] = (*value >> 8) & 0xFF; // 高字节
    buffer[1] = *value & 0xFF;        // 低字节
}
// 解析时（data -> int16_t）
static inline int16_t DecodeS16Data(uint8_t *src)
{
    return (int16_t)((src[0] << 8) | src[1]);
}
