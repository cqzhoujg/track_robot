//
// Created by zhoujg on 19-10-23.
//

#include "CMotor.h"
#include <ros/ros.h>


CMotor::CMotor()
{
    m_nMode = PV;
}

/*************************************************
Function: CMotor::SetMotorId
Description: 根据nodeID 设置电机的各种ID
Input: UINT nNodeId, 节点ID
Output: void
Others: void
**************************************************/
void CMotor::SetMotorId(UINT nNodeId)
{
    m_unNodeId = nNodeId;
    m_unSdoTxCobId = 0x600 + nNodeId;
    m_unSdoRxCobId = 0x580 + nNodeId;
    m_unPdoTxCobId = 0x300 + nNodeId;
    m_unPdoRxCobId = 0x280 + nNodeId;
    m_unHeartBeatId = 0x700 + nNodeId;
    m_unEmergencyId = 0x080 + nNodeId;
}

/*************************************************
Function: CMotor::SetMotorControlMode
Description: 设置电机的控制模式
Input: int nMode, 运动模式
Output: void
Others: void
**************************************************/
void CMotor::SetMotorControlMode(int nMode)
{
    m_nMode = nMode;
}

/*************************************************
Function: CMotor::InitMotorCmdPack
Description: 打包电机初始化 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::InitMotorCmdPack(CanFrame *canFrames)
{
    int nCount = 0;
    CanFrame *canFrameTemp = canFrames;

    nCount = EnableMotorCmdPack(canFrameTemp);
    canFrameTemp = canFrames + nCount;

    nCount += SportModeCmdPack(canFrameTemp);
    canFrameTemp = canFrames + nCount;

    nCount += QuickStopModeCmdPack(canFrameTemp);
    canFrameTemp = canFrames + nCount;

    nCount += StopModeCmdPack(canFrameTemp);
    canFrameTemp = canFrames + nCount;

    nCount += StopDecelerationCmdPack(canFrameTemp);
    canFrameTemp = canFrames + nCount;

    nCount += MapPdoCmdPack(canFrameTemp);
    canFrameTemp = canFrames + nCount;

    nCount += SetMasterHeartBeatCmdPack(canFrameTemp);

    return nCount;
}

/*************************************************
Function: CMotor::InitMotorCheckDataPack
Description: 打包电机初始化对应的接收 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::InitMotorCheckDataPack(CanFrame *canFrames)
{
    int nCount = 0;
    CanFrame *canFrameTemp = canFrames;

    nCount = EnableMotorCheckDataPack(canFrameTemp);
    canFrameTemp = canFrames + nCount;

    nCount += SportModeCheckDataPack(canFrameTemp);
    canFrameTemp = canFrames + nCount;

    nCount += QuickStopModeCheckDataPack(canFrameTemp);
    canFrameTemp = canFrames + nCount;

    nCount += StopModeCheckDataPack(canFrameTemp);
    canFrameTemp = canFrames + nCount;

    nCount += StopDecelerationCheckDataPack(canFrameTemp);
    canFrameTemp = canFrames + nCount;

    nCount += MapPdoCheckDataPack(canFrameTemp);
    canFrameTemp = canFrames + nCount;

    nCount += SetMasterHeartBeatCheckCmdPack(canFrameTemp);

    return nCount;
}

/*************************************************
Function: CMotor::EnableMotorCmdPack
Description: 打包电使能 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::EnableMotorCmdPack(CanFrame *canFrames)
{
    int nCount = 0;
    SetSdoCanFrame(canFrames, SdoControlWords[ENABLE_VOLTAGE]);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlWords[SWITCH_ON]);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlWords[ENABLE_OPERATION]);
    nCount++;

    return nCount;
}

/*************************************************
Function: CMotor::EnableMotorCheckDataPack
Description: 打包电机使能对应的接收 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::EnableMotorCheckDataPack(CanFrame *canFrames)
{
    int nCount = 0;
    SetSdoCanFrame(canFrames, SdoControlCheckWords[ENABLE_VOLTAGE], true);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlCheckWords[SWITCH_ON], true);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlCheckWords[ENABLE_OPERATION], true);
    nCount++;

    return nCount;
}

/*************************************************
Function: CMotor::SportModeCmdPack
Description: 打包设置电机运动模式 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::SportModeCmdPack(CanFrame *canFrames)
{
    if(m_nMode == PV)
        SetSdoCanFrame(canFrames, SdoControlWords[OPERATION_MODE_PV]);
    else if(m_nMode == PP)
        SetSdoCanFrame(canFrames, SdoControlWords[OPERATION_MODE_PP]);

    return 1;
}

/*************************************************
Function: CMotor::SportModeCheckDataPack
Description: 打包设置电机运动模式对应的接收 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::SportModeCheckDataPack(CanFrame *canFrames)
{
    if(m_nMode == PV)
        SetSdoCanFrame(canFrames, SdoControlCheckWords[OPERATION_MODE_PV], true);
    else if(m_nMode == PP)
        SetSdoCanFrame(canFrames, SdoControlCheckWords[OPERATION_MODE_PP], true);

    return 1;
}

/*************************************************
Function: CMotor::SetAccelerationCmdPack
Description: 打包设置电机加速度 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::SetAccelerationCmdPack(CanFrame *canFrames, double dAcceleration)
{
    if(m_nMode == PP)
    {
        CanopenSdoCmdWords ControlWord = SdoControlWords[T_ACCELERATION];
        int nMotorAcceleration = 5;

        ControlWord.Parameter += nMotorAcceleration;

        SetSdoCanFrame(canFrames, ControlWord);
    }
    if(m_nMode == PV)
    {
        CanopenSdoCmdWords ControlWord = SdoControlWords[T_ACCELERATION];
        int nMotorAcceleration = int((dAcceleration*MOTOR_TRANSMISSION*1000)/(M_PI*actual_tire_diameter));

        ControlWord.Parameter += nMotorAcceleration;

        SetSdoCanFrame(canFrames, ControlWord);
    }
    return 1;
}

/*************************************************
Function: CMotor::SetAccelerationCheckDataPack
Description: 打包设置电机加速度对应的接收 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::SetAccelerationCheckDataPack(CanFrame *canFrames)
{
    CanopenSdoCmdWords ControlWord = SdoControlCheckWords[T_ACCELERATION];
    SetSdoCanFrame(canFrames, ControlWord, true);
    return 1;
}

/*************************************************
Function: CMotor::SetDecelerationCmdPack
Description: 打包设置电机减速度 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::SetDecelerationCmdPack(CanFrame *canFrames, double dDeceleration)
{
    if(m_nMode == PP)
    {
        CanopenSdoCmdWords ControlWord = SdoControlWords[T_DECELERATION];
        int nMotorDeceleration = 5;

        ControlWord.Parameter += nMotorDeceleration;

        SetSdoCanFrame(canFrames, ControlWord);
    }
    if(m_nMode == PV)
    {
        CanopenSdoCmdWords ControlWord = SdoControlWords[T_DECELERATION];
        int nMotorDeceleration = int((dDeceleration*MOTOR_TRANSMISSION*1000)/(M_PI*actual_tire_diameter));

        ControlWord.Parameter += nMotorDeceleration;

        SetSdoCanFrame(canFrames, ControlWord);
    }
    return 1;
}

/*************************************************
Function: CMotor::SetDecelerationCheckDataPack
Description: 打包设置电机减速度对应的接收 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::SetDecelerationCheckDataPack(CanFrame *canFrames)
{
    CanopenSdoCmdWords ControlWord = SdoControlCheckWords[T_DECELERATION];
    SetSdoCanFrame(canFrames, ControlWord, true);
    return 1;
}

/*************************************************
Function: CMotor::SetNewPosCmdPack
Description: 打包设置位置模式下新位置点的 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
       double dDistance, 目标距离
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::SetNewPosCmdPack(CanFrame *canFrames, int nType, double dDistance)
{
    CanopenSdoCmdWords ControlWord;
    int nTargetPulse = 0, nCount = 0;

    nTargetPulse = int((dDistance*1000/(M_PI*actual_tire_diameter))*PULSES_OF_MOTOR_ONE_TURN*MOTOR_TRANSMISSION);
    ROS_DEBUG("Target distance:%f and Target pulse:%d and Diameter:%f",dDistance,nTargetPulse,actual_tire_diameter);

    ControlWord = SdoControlWords[SET_PP_NEW_POS];
    ControlWord.Parameter += nTargetPulse;

    SetSdoCanFrame(canFrames, ControlWord);
    canFrames++;
    nCount++;
    if(nType == RELATIVE)
        SetSdoCanFrame(canFrames, SdoControlWords[SET_PP_NEW_POS_RELATIVE]);
    else if(nType == ABSOLUTE)
        SetSdoCanFrame(canFrames, SdoControlWords[SET_PP_NEW_POS_ABSOLUTE]);
    nCount++;

    canFrames++;
    SetSdoCanFrame(canFrames, SdoControlWords[ENABLE_OPERATION]);
    nCount++;

    return nCount;
}

/*************************************************
Function: CMotor::SetDecelerationCheckDataPack
Description: 打包设置电机减速度对应的接收 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::SetNewPosCheckDataPack(CanFrame *canFrames, int nType)
{
    int nCount = 0;

    SetSdoCanFrame(canFrames, SdoControlCheckWords[SET_PP_NEW_POS], true);
    canFrames++;
    nCount++;
    if(nType == RELATIVE)
        SetSdoCanFrame(canFrames, SdoControlCheckWords[SET_PP_NEW_POS_RELATIVE], true);
    else if(nType == ABSOLUTE)
        SetSdoCanFrame(canFrames, SdoControlCheckWords[SET_PP_NEW_POS_ABSOLUTE], true);
    nCount++;

    return nCount;
}

/*************************************************
Function: CMotor::SetSpeedSdoCmdPack
Description: 打包通过sdo设置电机目标速度 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::SetSpeedSdoCmdPack(CanFrame *canFrames, double dSpeed)
{
    CanopenSdoCmdWords ControlWord;
    if(m_nMode == PV)
    {
        ControlWord = SdoControlWords[PV_TARGET_SPEED_SDO];
    }
    else
    {

        ControlWord = SdoControlWords[PP_TARGET_SPEED_SDO];
    }

    int nMotorSpeed = TransSpeed(dSpeed);

    ControlWord.Parameter += nMotorSpeed;
    SetSdoCanFrame(canFrames, ControlWord);

    return 1;
}

/*************************************************
Function: CMotor::SetSpeedPdoCmdPack
Description: 打包通过PDO设置电机目标速度 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::SetSpeedPdoCmdPack(CanFrame *canFrames, double dSpeed)
{
    uint32_t unMotorSpeed = 0;
    uint32_t unMotorSpeedTemp = 0;
    unMotorSpeedTemp = (uint32_t)TransSpeed(dSpeed);

    unMotorSpeed += (unMotorSpeedTemp & 0xFF00) >> 8;
    unMotorSpeed += (unMotorSpeedTemp & 0x00FF) << 8;

    SetPdoCanFrame(canFrames, m_unPdoTxCobId, 2,unMotorSpeed);

    return 1;
}

/*************************************************
Function: CMotor::GetMotorTempPack
Description: 打包获取电机伺服温度 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::GetMotorTempPack(CanFrame *canFrames)
{
    SetSdoCanFrame(canFrames, SdoControlWords[GET_TEMPERATURE]);
    return 1;
}

/*************************************************
Function: CMotor::MapPdoCmdPack
Description: 打包电机PDO映射 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::MapPdoCmdPack(CanFrame *canFrames)
{
    int nCount = 0;
    //映射60FF 为300+ID
    SetSdoCanFrame(canFrames, SdoControlWords[RPDO_PARM_1401_01_INIT]);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlWords[RPDO_PARM_1401_02]);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlWords[RPDO_MAP_1601_00_SET_0]);
    canFrames++;
    nCount++;
    if(m_nMode == PP)
    {
        SetSdoCanFrame(canFrames, SdoControlWords[RPDO_MAP_1601_01_PP]);
    }
    else
    {
        SetSdoCanFrame(canFrames, SdoControlWords[RPDO_MAP_1601_01_PV]);
    }
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlWords[RPDO_MAP_1601_00_SET_1]);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlWords[RPDO_PARM_1401_01_DONE]);
    canFrames++;
    nCount++;

    //映射606C 实时速度, 6041 伺服状态, 6063 实际位置,一起映射到281 282当中
    SetSdoCanFrame(canFrames, SdoControlWords[TPDO_PARM_1801_01_INIT]);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlWords[TPDO_PARM_1801_02]);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlWords[TPDO_MAP_1A01_00_INIT]);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlWords[TPDO_MAP_1A01_01]);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlWords[TPDO_MAP_1A01_02]);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlWords[TPDO_MAP_1A01_03]);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlWords[TPDO_MAP_1A01_00_DONE]);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlWords[TPDO_PARM_1801_01_DONE]);
    nCount++;

    return nCount;
}

/*************************************************
Function: CMotor::MapPdoCheckDataPack
Description: 打包电机PDO映射对应的接收 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::MapPdoCheckDataPack(CanFrame *canFrames)
{
    int nCount = 0;

    //映射60FF 为300+ID
    SetSdoCanFrame(canFrames, SdoControlCheckWords[RPDO_PARM_1401_01_INIT], true);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlCheckWords[RPDO_PARM_1401_02], true);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlCheckWords[RPDO_MAP_1601_00_SET_0], true);
    canFrames++;
    nCount++;
    if(m_nMode == PP)
    {
        SetSdoCanFrame(canFrames, SdoControlCheckWords[RPDO_MAP_1601_01_PP], true);
    }
    else
    {
        SetSdoCanFrame(canFrames, SdoControlCheckWords[RPDO_MAP_1601_01_PV], true);
    }
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlCheckWords[RPDO_MAP_1601_00_SET_1], true);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlCheckWords[RPDO_PARM_1401_01_DONE], true);
    canFrames++;
    nCount++;

    //映射606C 实时速度, 6041 伺服状态, 6063 实际位置,一起映射到281 282当中
    SetSdoCanFrame(canFrames, SdoControlCheckWords[TPDO_PARM_1801_01_INIT], true);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlCheckWords[TPDO_PARM_1801_02], true);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlCheckWords[TPDO_MAP_1A01_00_INIT], true);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlCheckWords[TPDO_MAP_1A01_01], true);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlCheckWords[TPDO_MAP_1A01_02], true);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlCheckWords[TPDO_MAP_1A01_03], true);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlCheckWords[TPDO_MAP_1A01_00_DONE], true);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlCheckWords[TPDO_PARM_1801_01_DONE], true);
    nCount++;

    return nCount;
}

/*************************************************
Function: CMotor::QuickStopModeCmdPack
Description: 打包设置电机快速停止 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::QuickStopModeCmdPack(CanFrame *canFrames)
{
    int nCount = 0;

    SetSdoCanFrame(canFrames, SdoControlWords[QUICK_STOP_MODE_SCRAM]);
    nCount++;
    return nCount;
}

/*************************************************
Function: CMotor::QuickStopModeCheckDataPack
Description: 打包设置电机快速停止模式对应的接收 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::QuickStopModeCheckDataPack(CanFrame *canFrames)
{
    int nCount = 0;

    SetSdoCanFrame(canFrames, SdoControlCheckWords[QUICK_STOP_MODE_SCRAM], true);
    nCount++;
    return nCount;
}

/*************************************************
Function: CMotor::QuickStopModeCmdPack
Description: 打包设置电机快速停止 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::StopModeCmdPack(CanFrame *canFrames)
{
    int nCount = 0;

    SetSdoCanFrame(canFrames, SdoControlWords[STOP_MODE_SLOW_DOWN]);
    nCount++;
    return nCount;
}

/*************************************************
Function: CMotor::QuickStopModeCheckDataPack
Description: 打包设置电机快速停止模式对应的接收 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::StopModeCheckDataPack(CanFrame *canFrames)
{
    int nCount = 0;

    SetSdoCanFrame(canFrames, SdoControlCheckWords[STOP_MODE_SLOW_DOWN], true);
    nCount++;
    return nCount;
}

/*************************************************
Function: CMotor::QuickStopModeCmdPack
Description: 打包设置电机快速停止 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::StopDecelerationCmdPack(CanFrame *canFrames)
{
    CanopenSdoCmdWords ControlWord = SdoControlWords[STOP_MODE_DECELERATION];
    int nMotorDeceleration = 8;

    ControlWord.Parameter += nMotorDeceleration;

    SetSdoCanFrame(canFrames, ControlWord);

    return 1;
}

/*************************************************
Function: CMotor::QuickStopModeCheckDataPack
Description: 打包设置电机快速停止模式对应的接收 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::StopDecelerationCheckDataPack(CanFrame *canFrames)
{
    int nCount = 0;

    SetSdoCanFrame(canFrames, SdoControlCheckWords[STOP_MODE_DECELERATION], true);
    nCount++;
    return nCount;
}

/*************************************************
Function: CMotor::QuickStopCmdPack
Description: 打包电机快速停止 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::QuickStopCmdPack(CanFrame *canFrames)
{
    int nCount = 0;

    SetSdoCanFrame(canFrames, SdoControlWords[QUICK_STOP]);
    nCount++;
    return nCount;
}

/*************************************************
Function: CMotor::StopCmdPack
Description: 打包电机快速停止 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::StopCmdPack(CanFrame *canFrames)
{
    int nCount = 0;

    SetSdoCanFrame(canFrames, SdoControlWords[STOP_HALT]);
    nCount++;
    return nCount;
}

/*************************************************
Function: CMotor::ClearWarnPack
Description: 打包清除电机报警状态的 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::ClearWarnPack(CanFrame *canFrames)
{
    int nCount = 0;

    SetSdoCanFrame(canFrames, SdoControlWords[CLEAR_WARN]);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlWords[ENABLE_OPERATION]);
    nCount++;
    return nCount;
}

/*************************************************
Function: CMotor::PackClearWarnCheckCmd
Description: 打包清除电机报警对应的接收 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::ClearWarnCheckCmdPack(CanFrame *canFrames)
{
    int nCount = 0;

    SetSdoCanFrame(canFrames, SdoControlCheckWords[CLEAR_WARN], true);
    canFrames++;
    nCount++;
    SetSdoCanFrame(canFrames, SdoControlCheckWords[ENABLE_OPERATION], true);
    nCount++;
    return nCount;
}

/*************************************************
Function: CMotor::GetErrorCodeCmdPack
Description: 打包获取电机报警状态字的 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::GetErrorCodeCmdPack(CanFrame *canFrames)
{
    int nCount = 0;

    SetSdoCanFrame(canFrames, SdoControlWords[ERROR_CODE]);
    nCount++;
    return nCount;
}

/*************************************************
Function: CMotor::SetMasterHeartBeatCmdPack
Description: 打包设置电机主站心跳的 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::SetMasterHeartBeatCmdPack(CanFrame *canFrames)
{
    int nCount = 0;

    SetSdoCanFrame(canFrames, SdoControlWords[SET_MASTER_HEART_BEAT]);
    nCount++;
    return nCount;
}

/*************************************************
Function: CMotor::SetMasterHeartBeatCheckCmdPack
Description: 打包设置电机心跳对应的接收 CAN 报文
Input: CanFrame *canFrames, USBCAN报文指针
Output: int nCount, 包中CAN帧数据的数量
Others: void
**************************************************/
int CMotor::SetMasterHeartBeatCheckCmdPack(CanFrame *canFrames)
{
    int nCount = 0;

    SetSdoCanFrame(canFrames, SdoControlCheckWords[SET_MASTER_HEART_BEAT], true);
    nCount++;
    return nCount;
}

/*************************************************
Function: CMotor::SetSdoCanFrame
Description: 将SDO指令封装至CAN帧当中
Input: CanFrame *canFrames, USBCAN报文指针; CanopenSdoCmdWords CmdData,sdo控制字; bool bIsCheck,是否为接收检测报文
Output: void
Others: void
**************************************************/
void CMotor::SetSdoCanFrame(CanFrame *canFrame, CanopenSdoCmdWords CmdData, bool bIsCheck)
{
    if(!bIsCheck)
        canFrame->ID = m_unSdoTxCobId;
    else
        canFrame->ID = m_unSdoRxCobId;

    /*只使用标准数据帧*/
    canFrame->RemoteFlag = 0;
    canFrame->ExternFlag = 0;
    canFrame->SendType = 0;

    if(CmdData.IsAddId)
    {
        CmdData.Parameter += m_unNodeId;
    }

    canFrame->DataLen = CmdData.DataLen;

    canFrame->Data[0] = CmdData.Cmd_byte;

    canFrame->Data[1] = (BYTE)CmdData.Index;
    canFrame->Data[2] = (BYTE)(CmdData.Index>>8);
    canFrame->Data[3] = CmdData.Sub_Index;

    canFrame->Data[4] = (uint8_t)CmdData.Parameter;
    canFrame->Data[5] = (uint8_t)(CmdData.Parameter >> 8);
    canFrame->Data[6] = (uint8_t)(CmdData.Parameter >> 16);
    canFrame->Data[7] = (uint8_t)(CmdData.Parameter >> 24);
}

/*************************************************
Function: CMotor::PackInitMotorCmd
Description: 将PDO指令封装至CAN帧当中
Input: CanFrame *canFrames, USBCAN报文指针; UINT ID,CAN帧ID; BYTE DataLen,CAN帧数据长度; ULL frameData,CAN帧数据
Output: void
Others: void
**************************************************/
void CMotor::SetPdoCanFrame(CanFrame *canFrame, UINT ID, BYTE DataLen, ULL frameData)
{
    canFrame->ID = ID;
    /*只使用标准数据帧*/
    canFrame->RemoteFlag = 0;
    canFrame->ExternFlag = 0;
    canFrame->SendType = 0;

    canFrame->DataLen = DataLen;
    for(int i=DataLen-1; i >= 0; i--)
    {
        canFrame->Data[i] = BYTE(frameData & 0xFF);
        frameData >>= 8;
    }
}

/*************************************************
Function: CMotor::UpdateMotorStatus
Description: 更新电机状态：位置 温度 当前实际速度
Input: CanFrame &canFrames, USBCAN报文
Output: void
Others: void
**************************************************/
void CMotor::UpdateMotorStatus(CanFrame &canFrame)
{
    //电机当前位置
    uint32_t unMotorPos = 0;
    unMotorPos = (unMotorPos+canFrame.szData[7]) << 8;
    unMotorPos = (unMotorPos+canFrame.szData[6]) << 8;
    unMotorPos = (unMotorPos+canFrame.szData[5]) << 8;
    unMotorPos = (unMotorPos+canFrame.szData[4]);
    m_MotorStatus.nCurrentPos = unMotorPos;

    //电机伺服状态
    int16_t nStatus = 0;
    nStatus = (nStatus+canFrame.szData[3]) << 8;
    nStatus = (nStatus+canFrame.szData[2]);
    m_MotorStatus.nServoStatus = nStatus;

    //电机实际速度
    int16_t nRealSpeed = 0;
    nRealSpeed = (nRealSpeed+canFrame.szData[1]) << 8;
    nRealSpeed = (nRealSpeed+canFrame.szData[0]);
    m_MotorStatus.dActual_speed = TransSpeed(nRealSpeed);
}

/*************************************************
Function: CMotor::ManageSdoInfo
Description: 处理 SDO
Input: CanFrame &canFrame,CAN数据帧
Output: 电机状态str
Others: void
**************************************************/
void CMotor::ManageSdoInfo(CanFrame &canFrame)
{
    if(canFrame.Data[0] == 0x43 && canFrame.Data[1] == 0x0B && canFrame.Data[2] == 0x20)
    {
        m_MotorStatus.sErr = GetMotorErrorWords(canFrame);
    }
}

/*************************************************
Function: CMotor::GetMotorErrorWords
Description: 获取电子状态字
Input: CanFrame &canFrame,CAN数据帧
Output: 电机状态str
Others: void
**************************************************/
string CMotor::GetMotorErrorWords(CanFrame &canFrame)
{
    //Error Code Info
    uint32_t nErrorCodeInfo = 0;
    nErrorCodeInfo = (nErrorCodeInfo+canFrame.szData[7]) << 8;
    nErrorCodeInfo = (nErrorCodeInfo+canFrame.szData[6]) << 8;
    nErrorCodeInfo = (nErrorCodeInfo+canFrame.szData[5]) << 8;
    nErrorCodeInfo = (nErrorCodeInfo+canFrame.szData[4]);
    if(nErrorCodeInfo == 0)
        return "normal";

    string sErrorWords = "Motor_" + to_string(m_unNodeId) + ":";
    sErrorWords += (nErrorCodeInfo & 0x00000001) == 0 ? "" : " Sys Error;";
    sErrorWords += (nErrorCodeInfo & 0x00000002) == 0 ? "" : " Over Load Warn;";
    sErrorWords += (nErrorCodeInfo & 0x00000004) == 0 ? "" : " Parm Error;";
    sErrorWords += (nErrorCodeInfo & 0x00000008) == 0 ? "" : " Under Voltage Warn;";
    sErrorWords += (nErrorCodeInfo & 0x00000010) == 0 ? "" : " Over Voltage Warn;";
    sErrorWords += (nErrorCodeInfo & 0x00000020) == 0 ? "" : " Long Time Over Load Warn;";
    sErrorWords += (nErrorCodeInfo & 0x00000040) == 0 ? "" : " Over Current Warn;";
    sErrorWords += (nErrorCodeInfo & 0x00000080) == 0 ? "" : " Encoder Error;";
    sErrorWords += (nErrorCodeInfo & 0x00000100) == 0 ? "" : " Position Error;";
    sErrorWords += (nErrorCodeInfo & 0x00000200) == 0 ? "" : " Speed Error;";
    sErrorWords += (nErrorCodeInfo & 0x00000400) == 0 ? "" : " Level One High Temp Warn;";
    sErrorWords += (nErrorCodeInfo & 0x00000800) == 0 ? "" : " Level Two High Temp Warn;";
    sErrorWords += (nErrorCodeInfo & 0x00001000) == 0 ? "" : " Over Speed Warn;";
    sErrorWords += (nErrorCodeInfo & 0x00002000) == 0 ? "" : " Flash Error;";
    sErrorWords += (nErrorCodeInfo & 0x00004000) == 0 ? "" : " Current Zero Error;";
    sErrorWords += (nErrorCodeInfo & 0x00008000) == 0 ? "" : " Limit PosWarn;";

    return sErrorWords;
}

/*************************************************
Function: CMotor::TransSpeed
Description: 将里程速度转换成伺服的脉冲速度
Input: double dSpeed, 机器人运动速度(单位:m/s)
Output: 电机转速(单位:RMP)
Others: void
**************************************************/
int CMotor::TransSpeed(double dSpeed)
{
    return int((dSpeed*1000/(M_PI*actual_tire_diameter))*60*MOTOR_TRANSMISSION);
}

/*************************************************
Function: CMotor::TransSpeed
Description: 将伺服的脉冲速度转换成里程速度
Input: int nPulses, 电机转速(单位:RMP)
Output: 机器人运动速度(单位:m/s)
Others: void
**************************************************/
double CMotor::TransSpeed(int16_t nPulses)
{
    return ((double)nPulses)/(60*MOTOR_TRANSMISSION*1000)*M_PI*actual_tire_diameter;
}

/*************************************************
Function: CMotor::TransMileToPulse
Description: 将距离转换为伺服脉冲值
Input: void
Output: void
Others: void
**************************************************/
int CMotor::TransMileToPulse(double dDistance)
{
    return int(( (dDistance*1000) / (M_PI*actual_tire_diameter) )*PULSES_OF_MOTOR_ONE_TURN*MOTOR_TRANSMISSION );
}

CMotor::~CMotor()
{

}