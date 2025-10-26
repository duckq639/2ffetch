#ifndef DJ_MOTOR_H
#define DJ_MOTOR_H
/*=======================================代码说明=====================================================*/
/*驱动适用于M3508和M2006类型电机,配合C610和C620电调使用
 * func函数置于定时器中循环执行,有多台电机时,外套for循环执行
 * 默认启用CAN2通信,cubemx的默认CAN2引脚是错误的,手动启用PB5,PB6
 */
/*=======================================头文件包含=====================================================*/
#include "can.h"
#include "stdbool.h"
#include "string.h"

#include "mathFunc.h"
#include "pid.h"
/*=========================================宏定义==========================================================*/
#ifndef USE_M2006_NUM //  定义是否使用M2006电机的数量，默认为4
#define USE_M2006_NUM 2
#endif
#ifndef USE_M3508_NUM //  定义是否使用M3508电机的数量，默认为0
#define USE_M3508_NUM 1
#endif
#define USE_DJ_NUM (USE_M2006_NUM + USE_M3508_NUM) //  定义使用的电机总数，为M2006和M3508电机数量之和
#define M2006_RATIO (float)36
#define M3508_RATIO (float)19.20321

#ifndef DJ_CAN_CHANNEL
#define DJ_CAN_CHANNEL hcan2
#endif

#define ZERO_DISTANCE_THRESHOLD 5      //  定义零距离阈值，用于判断是否到达目标位置
#define SETLOCK_MAXDIS_THRESHOLD 10    //  定义设置锁定的最大距离阈值，用于判断是否可以锁定
#define POSETOIN_DISTANCE_THRESHOLD 30 //  定义位置近目标距离阈值，用于判断是否到达目标位置附近
#define POSETOIN_BIAS_COMP 1           // 由于偏移量等于裕度,所以设置一个偏移补偿量
/*=============================================数据结构=======================================================*/
typedef enum
{
    CAN_FSTATE_IDLE, // 空闲
    CAN_FSTATE_BUSY, // 正在收/发
    CAN_FSTATE_WAIT, // 等待中
    CAN_FSTATE_ERROR // 出错
} CAN_Frame_State;

typedef struct
{
    CAN_Frame_State CAN_FState;
    CAN_HandleTypeDef *hcan;
    CAN_TxHeaderTypeDef CAN_TxHeader;
    CAN_RxHeaderTypeDef CAN_RxHeader;
    uint8_t Data[8];
} CAN_Frame;

// CAN收发

typedef enum
{
    DJM2006,
    DJM3508,
} DJMotorType;

typedef struct
{
    volatile int16_t current;       // 电机的给定电流（控制器输出给电机的期望电流，单位是mA，C620反馈是-16384~16384）
    volatile int16_t speed;         // 电机转速
    volatile float angle;           // 电机转子角度（范围 0~8191，对应 0~360°）
    volatile float currentA;        // 电机实际相电流（A）
    volatile int16_t pulseread;     // 编码器当前读数（随转子转动变化，0~8191循环）
    volatile int32_t pulsetotal;    // 累计脉冲数
    volatile int16_t pulsedistance; // 本次转动的脉冲增量（差分值，用于计算瞬时速度或位移）
    volatile int8_t temperature;    // 电机温度（°C），一般是电机驱动板/绕组的温度
} DJMotorValue;                     /*电机运行参数*/

typedef struct
{
    uint16_t pulsePerRound; // 电机一圈脉冲数
    float reductionRatio;   // 电机减速比
    float gearRatio;        // 机构减速比
    uint16_t currentLimit;
    uint16_t StdID; // CAN参数
} DJMotorParam;     // 电机静态参数,包含CAN参数

typedef enum //  电机模式，用于指定电机的操作模式
{
    Disable,  /*失能状态*/
    Enable,   /*使能状态*/
    Speed,    /*速度模式*/
    Position, /*位置模式*/
    Zero,     /*寻零模式*/
} DJMotorMode;

typedef struct
{
    volatile int32_t pulseLock;   // 脉冲锁定值，用来做零点对齐或位置参考
    volatile uint32_t lastRxTime; // 上一次收到电机反馈的时间戳
    uint16_t zeroCnt;             // 寻零计数器，用来统计寻零过程走了多少步/次数
    uint16_t deadZoneCnt;
    uint16_t pawLockCnt;  // 爪位锁定计数器
    uint32_t maxDistance; // 运行最大行程（脉冲数，用于限位）
    int16_t dirCnt;       // 方向计数器,根据distance的变化辅助确定运动方向
} DJMotorConter;          // 运行计数器

typedef struct // 数据限制
{
    // 标志位限制
    bool isRPMLimit;      // 速度模式下的速度限制
    bool isPosAngleLimit; // 位置模式下的角度限制
    bool posRPMFlag;      // 位置模式下的速度环pid计算限制标志位
    bool isCurrentLimit;  // 电流限制判断
    bool isStuck;         // 是否堵转
    bool deadZoneEnable;  // 是否启用角度死区控制

    // 数值限制
    float maxAngle; // 最大角度
    float minAngle; // 最小角度

    int32_t posRPMLimit; // 位置模式下的最大限制角度，这里实际上是对pid计算的中间量进行限制
    int16_t zeroSpeed;   // 寻零最大速度限制
    int16_t zeroCurrent; // 寻零最大电流
} DJMotorLimit;

typedef struct
{
    bool releaseWhenStuck; // 堵转后是否释放
    bool stuckDetected;    // 堵转标志
    bool timeoutDetected;  // 超时标志
    uint16_t stuckCount;   // 堵转计数
    uint16_t timeoutCount; // 超时计数
} DJMotorError;            // 电机异常反馈

typedef struct
{
    volatile bool isArrived;   // 是否到位
    volatile bool isZeroed;    // 寻零完成
    volatile bool isOvertime;  // 是否超时
    volatile bool isStuck;     // 是否堵转
    volatile bool setZeroFlag; // 是否将当前位置置为0：
    volatile bool findZeroDir; // 寻零方向标志位
    volatile int8_t dirFlag;   // 运动方向标志位,1为正方向
} DJMotorStatus;               // 电机执行情况

typedef struct
{
    // 基础配置
    DJMotorMode motorMode; // 电机工作模式
    DJMotorType motorType; // 电机类型标志

    // 运行控制
    volatile bool begin;

    // PID控制
    PIDtype posPID; // 位置PID
    PIDtype rpmPID; // 速度PID

    // 电机参数
    DJMotorParam param;    // 静态参数
    DJMotorValue valueSet; // 设定值
    DJMotorValue valueNow; // 当前值
    DJMotorValue valuePre; // 前一时刻值

    // 控制相关
    DJMotorConter conter; // 计数器
    DJMotorLimit limit;   // 限制参数
    DJMotorError error;   // 错误状态
    DJMotorStatus status; // 运行状态

} DJMotor, *DJMotorPtr;
/*=============================================类型声明=======================================================*/
extern DJMotor DJMotors[USE_DJ_NUM]; // 唯一指定调用API
extern int8_t IDlist[9];
extern CAN_Frame DJCAN_TxFrame;
extern CAN_Frame DJCAN_RxFrame;
/*=============================================函数声明=======================================================*/
void DJMotor_Init(DJMotorPtr DJMotorsp, uint16_t ID, DJMotorType mtype);
void DJMotor_Read_Cmd(CAN_RxHeaderTypeDef *rxhdr); // 放置在can中断回调函数中,在此函数之前先getmessage存入头
void DJMotor_Func();
#endif // DJ_MOTOR_H
