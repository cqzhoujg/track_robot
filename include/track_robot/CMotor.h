//
// Created by zhoujg on 19-10-23.
//

#ifndef PROJECT_CMOTOR_H
#define PROJECT_CMOTOR_H

#include <iostream>
#include <string.h>
#include <mutex>
#include <fcntl.h>
#include <cmath>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "CUsbCan.h"

using namespace std;
using namespace UsbCan;

//added by btrmg for adjust mileage 2020.01.07
extern double actual_tire_diameter;
extern double forward_diameter;
extern double back_diameter;
//added end

//50000个脉冲代表一圈,第一版机器人的传动比为 2.5 : 1,即电机2.5转对应轮胎1转,而
//#define MOTOR_TRANSMISSION 1 //步进电机传动比
#define MOTOR_TRANSMISSION 16.5 //伺服电机传动比
//#define TIRE_DIAMETER 148 //机器人轮胎直径(mm)
#define TIRE_DIAMETER 143 //机器人轮胎直径(mm)
//#define PULSES_OF_MOTOR_ONE_TURN 50000 //步进电机一转里程对应的脉冲数
#define PULSES_OF_MOTOR_ONE_TURN 16384 //伺服电机一转里程对应的脉冲数
#define GET_MOTOR_INFO_PDO_ID 0x080
#define MASTER_HEART_BEAT_ID 0x77F
#define MOTOR_INITIAL_POS_VALUE 0 //电机上电初始位置值
#define AUTO_RUN_LOCATION_TRIM 0.001     //用于速度模式下，auto run时的目标距离修正值

/********************************************************************************************
 * 1.心跳
 * 2.断电
 * 3.上电
 * 4.执行
 * 5.加速度
 * 6.减速度
 * 7.目标速度
 * 8.急停
 * 9.PDO映射
 * 10.状态读取
 *
 * SDO的命令字： 主站 client 命令字
 * 0～7个字节：0字节表示是否指明数据长度;1字节表示是否加速传输;2~3字节,0和1字节都为1时才有效,表示无意义数据的个数;
 * 4字节无效;5~7字节,client固定值从7字节到5字节分别为 1 0 0;
 * 如果不指定数据长度，则n无意义，n=0，那么命令字就等于00100010b,即0x22
 *      如果只使用1个字节，则无意义字节数为3(BYTE6-8)。n=11b,那么命令字就等于00101111，即0x2F
 *      如果只使用2个字节，则无意义字节数为2(BYTE7-8)。n=10b,那么命令字就等于00101011，即0x2B
 *      如果只使用3个字节，则无意义字节数为1(BYTE8)  。n=01b,那么命令字就等于00100111，即0x27
 *      如果要使用4个字节，则无意义字节数为0         。n=00b,那么命令字就等于00100011，即0x23
*********************************************************************************************/

typedef struct _tagCanopenSdoCmdWords
{
    uint8_t Cmd_byte;
    uint16_t Index;
    uint8_t Sub_Index;
    uint32_t Parameter;
    uint8_t DataLen;
    uint8_t IsAddId;
}CanopenSdoCmdWords, *PCanopenSdoCmdWords;

const CanopenSdoCmdWords SdoControlWords[40]=
{
    /*******************SDO***********************/
    /* 断电 指令*/
    {0X2B,0X6040,0X00,0X00000006,8,0},//0 ENABLE_VOLTAGE
    /* 上电 指令*/
    {0X2B,0X6040,0X00,0X00000007,8,0},//1 SWITCH_ON
    /* 使能 指令*/
    {0X2B,0X6040,0X00,0X0000000F,8,0},//2 ENABLE_OPERATION
    /* 停止 指令*/
    {0X2B,0X6040,0X00,0X0000010F,8,0},//3 STOP_HALT
    /* 速度模式 指令*/
    {0X2B,0X6060,0X00,0X00000003,8,0},//4 OPERATION_MODE_PV
    /* 位置模式 指令*/
    {0X2B,0X6060,0X00,0X00000001,8,0},//5 OPERATION_MODE_PP
    /* 加速度 指令*/
    {0X2B,0X6082,0X00,0X00000000,8,0},//6 T_ACCELERATION
    /* 减速度 指令*/
    {0X2B,0X6083,0X00,0X00000000,8,0},//7 T_DECELERATION
    /* PV 速度模式的目标速度 指令*/
    {0X2B,0X60FF,0X00,0X00000000,8,0},//8 PV_TARGET_SPEED_SDO
    /* PP target speed 设置位置模式下的目标速度*/
    {0X2B,0X607F,0X00,0X00000000,8,0},//9 PP_TARGET_SPEED_SDO

    /*用于速度模式 映射60FF ID 为300+NodeId 的PDO中*/
    {0X23,0X1401,0X01,0X80000300,8,1},//10  RPDO_PARM_1401_01_INIT
    {0X2F,0X1401,0X02,0X000000FF,8,0},//11 RPDO_PARM_1401_02
    {0X2F,0X1601,0X00,0X00000000,8,0},//12 RPDO_MAP_1601_00_SET_0
    {0X23,0X1601,0X01,0X60FF0010,8,0},//13 RPDO_MAP_1601_01_PV
    {0X23,0X1601,0X01,0X607F0010,8,0},//14 RPDO_MAP_1601_01_PP
    {0X2F,0X1601,0X00,0X00000001,8,0},//15 RPDO_MAP_1601_00_SET_1
    {0X23,0X1401,0X01,0X00000300,8,1},//16 RPDO_PARM_1401_01_DONE

    /* 映射606C 实时速度, 6041 伺服状态, 6063 实际位置,一起映射到ID为 280+NodeId 的PDO当中*/
    {0X23,0X1801,0X01,0X80000280,8,1},//17 TPDO_PARM_1801_01_INIT
    {0X2F,0X1801,0X02,0X00000001,8,0},//18 TPDO_PARM_1801_02
    {0X2F,0X1A01,0X00,0X00000000,8,0},//19 TPDO_MAP_1A01_00_INIT,
    {0X23,0X1A01,0X01,0X606C0010,8,0},//20 TPDO_MAP_1A01_01,
    {0X23,0X1A01,0X02,0X60410010,8,0},//21 TPDO_MAP_1A01_02,
    {0X23,0X1A01,0X03,0X60630020,8,0},//22 TPDO_MAP_1A01_03,
    {0X2F,0X1A01,0X00,0X00000003,8,0},//23 TPDO_MAP_1A01_00_DONE,
    {0X23,0X1801,0X01,0X00000280,8,1},//24 TPDO_PARM_1801_01_DONE,

    /* 6094 配置急停方式,0-没有减速 运动直接停止, 1-电机按照设定的减速度停止, 2-命令电机释放 */
    {0X2B,0X6094,0X00,0X00000000,8,0},//25 QUICK_STOP_MODE_SCRAM
    {0X2B,0X6094,0X00,0X00000001,8,0},//26 QUICK_STOP_MODE_SLOW_DOWN
    /* Quick stop 指令*/
    {0X2B,0X2003,0X00,0X00000001,8,0},//27 QUICK_STOP

    /* Clear warn 清除警告 指令*/
    {0X2B,0X6040,0X00,0X00000086,8,0},//28 CLEAR_WARN
    /* Error code 驱动器的故障代码 指令*/
    {0X40,0X200B,0X00,0X00000000,8,0},//29 ERROR_CODE

    /* Get temperature 获取驱动器温度*/
    {0X40,0X6091,0X00,0X00000000,8,0},//30 GET_TEMPERATURE
    /* Set Master HeartBeat 设置主站心跳几ID*/
    {0X23,0X1016,0X01,0X007F04B0,8,0},//31 SET_MASTER_HEART_BEAT

    /* 位置模式下控制指令*/
    {0X23,0X607A,0X00,0X00000000,8,0},//32 SET_PP_NEW_POS,
//    {0X2B,0X6040,0X00,0X0000001F,8,0},//33 SET_PP_NEW_POS_ENABLE
    {0X2B,0X6040,0X00,0X0000001F,8,0},//34 SET_PP_NEW_POS_RELATIVE
    {0X2B,0X6040,0X00,0X0000005F,8,0},//35 SET_PP_NEW_POS_ABSOLUTE

    /* halt stop 选项 配置急停方式,0-没有减速 运动直接停止, 1-电机按照设定的减速度停止, 2-命令电机释放 */
    {0X2B,0X6095,0X00,0X00000001,8,0},//36 STOP_MODE_SLOW_DOWN
    {0X2B,0X6069,0X00,0X00000000,8,0},//37 STOP_MODE_DECELERATION
};

const CanopenSdoCmdWords SdoControlCheckWords[40]=
{
    /*******************SDO***********************/
    /* 断电 指令*/
    {0X60,0X6040,0X00,0X00000000,8,0},//0 ENABLE_VOLTAGE
    /* 上电 指令*/
    {0X60,0X6040,0X00,0X00000000,8,0},//1 SWITCH_ON
    /* 使能 指令*/
    {0X60,0X6040,0X00,0X00000000,8,0},//2 ENABLE_OPERATION
    /* 停止 指令*/
    {0X60,0X6040,0X00,0X00000000,8,0},//3 STOP_HALT
    /* 速度模式 指令*/
    {0X60,0X6060,0X00,0X00000000,8,0},//4 OPERATION_MODE_PV
    /* 位置模式 指令*/
    {0X60,0X6060,0X00,0X00000000,8,0},//5 OPERATION_MODE_PP
    /* 加速度 指令*/
    {0X60,0X6082,0X00,0X00000000,8,0},//6 T_ACCELERATION
    /* 减速度 指令*/
    {0X60,0X6083,0X00,0X00000000,8,0},//7 T_DECELERATION
    /* 速度模式的目标速度 指令*/
    {0X60,0X60FF,0X00,0X00000000,8,0},//8 PV_TARGET_SPEED_SDO
    /* PP target speed 设置位置模式下的目标速度*/
    {0X60,0X607F,0X00,0X00000000,8,0},//9 PP_TARGET_SPEED_SDO

    /*映射60FF ID 为300+NodeId 的PDO中*/
    {0X60,0X1401,0X01,0X00000000,8,0},//10 RPDO_PARM_1401_01_INIT
    {0X60,0X1401,0X02,0X00000000,8,0},//11 RPDO_PARM_1401_02
    {0X60,0X1601,0X00,0X00000000,8,0},//12 RPDO_MAP_1601_00_SET_0
    {0X60,0X1601,0X01,0X00000000,8,0},//13 RPDO_MAP_1601_01_PV
    {0X60,0X1601,0X01,0X00000000,8,0},//14 RPDO_MAP_1601_01_PP
    {0X60,0X1601,0X00,0X00000000,8,0},//15 RPDO_MAP_1601_00_SET_1
    {0X60,0X1401,0X01,0X00000000,8,0},//16 RPDO_PARM_1401_01_DONE

    /* 映射606C 实时速度, 6041 伺服状态, 6063 实际位置,一起映射到ID为 280+NodeId 的PDO当中*/
    {0X60,0X1801,0X01,0X00000000,8,0},//17 TPDO_PARM_1801_01_INIT
    {0X60,0X1801,0X02,0X00000000,8,0},//18 TPDO_PARM_1801_02
    {0X60,0X1A01,0X00,0X00000000,8,0},//19 TPDO_MAP_1A01_00_INIT,
    {0X60,0X1A01,0X01,0X00000000,8,0},//20 TPDO_MAP_1A01_01,
    {0X60,0X1A01,0X02,0X00000000,8,0},//21 TPDO_MAP_1A01_02,
    {0X60,0X1A01,0X03,0X00000000,8,0},//22 TPDO_MAP_1A01_03,
    {0X60,0X1A01,0X00,0X00000000,8,0},//23 TPDO_MAP_1A01_00_DONE,
    {0X60,0X1801,0X01,0X00000000,8,0},//24 TPDO_PARM_1801_01_DONE,

    /* 6094 配置急停方式,0-没有减速 运动直接停止, 1-电机按照设定的减速度停止, 2-命令电机释放 */
    {0X60,0X6094,0X00,0X00000000,8,0},//25 QUICK_STOP_MODE_SCRAM
    {0X60,0X6094,0X00,0X00000000,8,0},//26 QUICK_STOP_MODE_SLOW_DOWN
    /* Quick stop 指令*/
    {0X60,0X2003,0X00,0X00000000,8,0},//27 QUICK_STOP

    /* Clear warn 清除警告 指令*/
    {0X60,0X6040,0X00,0X00000000,8,0},//28 CLEAR_WARN
    /* Error code 驱动器的故障代码 指令*/

    {0X60,0X200B,0X00,0X00000000,8,0},//29 ERROR_CODE
    /* Get temperature 获取驱动器温度*/
    {0X60,0X6091,0X00,0X00000000,8,0},//30 GET_TEMPERATURE
    /* Set Master HeartBeat 设置主站心跳几ID*/
    {0X60,0X1016,0X01,0X00000000,8,0},//31 SET_MASTER_HEART_BEAT

    /* 位置模式下控制指令*/
    {0X60,0X607A,0X00,0X00000000,8,0},//32 SET_PP_NEW_POS,
//    {0X60,0X6040,0X00,0X00000000,8,0},//34 SET_PP_NEW_POS_ENABLE
    {0X60,0X6040,0X00,0X00000000,8,0},//33 SET_PP_NEW_POS_RELATIVE
    {0X60,0X6040,0X00,0X00000000,8,0},//35 SET_PP_NEW_POS_ABSOLUTE

    /* halt stop 选项 配置急停方式,0-没有减速 运动直接停止, 1-电机按照设定的减速度停止, 2-命令电机释放 */
    {0X60,0X6095,0X00,0X00000000,8,0},//36 STOP_MODE_SLOW_DOWN
    {0X60,0X6069,0X00,0X00000000,8,0},//37 STOP_MODE_DECELERATION
};

typedef enum _tagCanOpenSdoCmd_
{
    ENABLE_VOLTAGE,     //0
    SWITCH_ON,          //1
    ENABLE_OPERATION,   //2
    STOP_HALT,          //3
    OPERATION_MODE_PV,  //4
    OPERATION_MODE_PP,  //5
    T_ACCELERATION,    //6
    T_DECELERATION,    //7
    PV_TARGET_SPEED_SDO,//8
    PP_TARGET_SPEED_SDO,//9

    RPDO_PARM_1401_01_INIT,//10
    RPDO_PARM_1401_02,  //11
    RPDO_MAP_1601_00_SET_0,  //12
    RPDO_MAP_1601_01_PV,  //13
    RPDO_MAP_1601_01_PP,  //14
    RPDO_MAP_1601_00_SET_1,  //15
    RPDO_PARM_1401_01_DONE,//16

    TPDO_PARM_1801_01_INIT,//17
    TPDO_PARM_1801_02,  //18
    TPDO_MAP_1A01_00_INIT,  //19
    TPDO_MAP_1A01_01,  //20
    TPDO_MAP_1A01_02,  //21
    TPDO_MAP_1A01_03,  //22
    TPDO_MAP_1A01_00_DONE,//23
    TPDO_PARM_1801_01_DONE,//24

    QUICK_STOP_MODE_SCRAM, //25
    QUICK_STOP_MODE_SLOW_DOWN,//26
    QUICK_STOP,//27

    CLEAR_WARN,//28
    ERROR_CODE,//29

    GET_TEMPERATURE,//30
    SET_MASTER_HEART_BEAT,//31

    /*位置模式下控制指令*/
    SET_PP_NEW_POS,//32
//    SET_PP_NEW_POS_ENABLE,//33
    SET_PP_NEW_POS_RELATIVE,//34
    SET_PP_NEW_POS_ABSOLUTE,//35
    STOP_MODE_SLOW_DOWN,//36
    STOP_MODE_DECELERATION,//37

}CanOpenSdoCmd;

typedef enum _tagSportMOde
{
    PP,
    PV
}SportMOde;

typedef enum _tagPPType
{
    RELATIVE,
    ABSOLUTE
}PPType;

typedef struct _tagMotorStatus
{
    double dTarget_speed = 0;   //目标速度
    double dActual_speed = 0;   //实际速度
    int16_t nServoStatus = 0;   //伺服状态
    int nCurrentPos = 0;        //实际位置
    string sErr = "normal";
}MotorStatus, *pMotorStatus;

class CMotor
{
public:
    CMotor();
    ~CMotor();
    void SetMotorId(UINT nNodeId);
    void SetMotorControlMode(int nMode);
    int InitMotorCmdPack(CanFrame *canFrames);
    int InitMotorCheckDataPack(CanFrame *canFrames);
    int EnableMotorCmdPack(CanFrame *canFrames);
    int EnableMotorCheckDataPack(CanFrame *canFrames);
    int SportModeCmdPack(CanFrame *canFrames);
    int SportModeCheckDataPack(CanFrame *canFrames);
    int SetAccelerationCmdPack(CanFrame *canFrames, double dAcceleration);
    int SetAccelerationCheckDataPack(CanFrame *canFrames);
    int SetDecelerationCmdPack(CanFrame *canFrames, double dDeceleration);
    int SetDecelerationCheckDataPack(CanFrame *canFrames);
    int SetNewPosCmdPack(CanFrame *canFrames, int nType, double dDistance);
    int SetNewPosCheckDataPack(CanFrame *canFrames, int nType);
    int SetSpeedSdoCmdPack(CanFrame *canFrames, double dSpeed);
    int SetSpeedPdoCmdPack(CanFrame *canFrames, double dSpeed);
    int GetMotorTempPack(CanFrame *canFrames);
    int MapPdoCmdPack(CanFrame *canFrames);
    int MapPdoCheckDataPack(CanFrame *canFrames);
    int QuickStopModeCmdPack(CanFrame *canFrames);
    int QuickStopModeCheckDataPack(CanFrame *canFrames);
    int StopModeCmdPack(CanFrame *canFrames);
    int StopModeCheckDataPack(CanFrame *canFrames);
    int StopDecelerationCmdPack(CanFrame *canFrames);
    int StopDecelerationCheckDataPack(CanFrame *canFrames);
    int QuickStopCmdPack(CanFrame *canFrames);
    int StopCmdPack(CanFrame *canFrames);
    int ClearWarnPack(CanFrame *canFrames);
    int ClearWarnCheckCmdPack(CanFrame *canFrames);
    int GetErrorCodeCmdPack(CanFrame *canFrames);
    int SetMasterHeartBeatCmdPack(CanFrame *canFrames);
    int SetMasterHeartBeatCheckCmdPack(CanFrame *canFrames);
    void SetSdoCanFrame(CanFrame *canFrame, CanopenSdoCmdWords CmdData, bool bIsCheck = false);
    void SetPdoCanFrame(CanFrame *canFrame, UINT ID, BYTE DataLen, ULL frameData);
    void UpdateMotorStatus(CanFrame &canFrame);
    void ManageSdoInfo(CanFrame &canFrame);
    string GetMotorErrorWords(CanFrame &canFrame);
    int TransSpeed(double dSpeed);
    double TransSpeed(int16_t nPulses);
    int TransMileToPulse(double dDistance);

    MotorStatus m_MotorStatus;
    UINT m_unNodeId;
    UINT m_unSdoTxCobId;
    UINT m_unSdoRxCobId;
    UINT m_unPdoTxCobId;
    UINT m_unPdoRxCobId;
    UINT m_unHeartBeatId;
    UINT m_unEmergencyId;
    int m_nMode;
};

#endif //PROJECT_CMOTOR_H
