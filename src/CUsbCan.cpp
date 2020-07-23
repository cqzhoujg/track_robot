/*************************************************
Copyright: wootion
Author: ZhouJinGang
Date: 2018-12-25
Description: USBCAN 功能函数二次封装
**************************************************/

#include <ros/ros.h>
#include "CUsbCan.h"
#include <string>
namespace UsbCan
{
CUsbCan::CUsbCan()
{
    SetCanSpeed(500000);
    m_bDeviceOpened = false;
}

CUsbCan::~CUsbCan()
{
    m_bDeviceOpened = false;
    VCI_ClearBuffer(VCI_USBCAN2, 0, 0);//清空buffer。
    VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
#if USE_CAN2_RT
    VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN2通道。
#endif
    VCI_CloseDevice(VCI_USBCAN2, 0);
}

/*************************************************
Function: CUsbCan::Init
Description:　USBCAN初始化功能封装
Input: void
Output: true  USBCAN设备初始化成功
        false USBCAN设备初始化失败
Others: void
**************************************************/
bool CUsbCan::Init()
{
    ROS_DEBUG("[CUsbCan][Init] begin...");
    DWORD ret = 0;
    if((ret=VCI_OpenDevice(VCI_USBCAN2,0,0))==1)//打开设备
    {
        ROS_DEBUG("[CUsbCan][Init] open device VCI_USBCAN2 success!");//打开设备成功
    }
    else
    {
        ROS_ERROR("[CUsbCan][Init] open device VCI_USBCAN2 error, ret = %d",ret);
        exit(1);
    }

    m_config.AccCode = 0;
    m_config.AccMask = 0xffffffff;
    m_config.Filter = 1;
    m_config.Mode = 0;
    m_config.Timing0 = UCHAR(m_unBaud & 0xff);
    m_config.Timing1 = UCHAR(m_unBaud >> 8);

    if (VCI_InitCAN(VCI_USBCAN2, 0, 0, &m_config) != 1)
    {
        ROS_ERROR("[CUsbCan][Init] VCI_InitCAN1 failed");
        VCI_CloseDevice(VCI_USBCAN2,0);
        return false;
    }
    ROS_DEBUG("[CUsbCan][Init] VCI_InitCAN1 succeeded");

//    VCI_ClearBuffer(VCI_USBCAN2, 0, 0);//清空buffer。

    if (VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1)
    {
        ROS_ERROR("[CUsbCan][Init] VCI_StartCAN1 failed");
        VCI_CloseDevice(VCI_USBCAN2,0);
        return false;
    }
    ROS_DEBUG("[CUsbCan][Init] VCI_StartCAN1 succeeded");

#if USE_CAN2_RT
    if (VCI_InitCAN(VCI_USBCAN2, 0, 1, &m_config) != 1)
    {
        ROS_ERROR("[CDeviceNetMaster][Init] VCI_InitCAN2 failed");
        VCI_CloseDevice(VCI_USBCAN2,0);
        return false;
    }
    ROS_DEBUG("[CDeviceNetMaster][Init] VCI_InitCAN2 succeeded");

    VCI_ClearBuffer(VCI_USBCAN2, 0, 1);//清空buffer。

    if (VCI_StartCAN(VCI_USBCAN2, 0, 1) != 1)
    {
        ROS_ERROR("[CDeviceNetMaster][Init] VCI_StartCAN2 failed");
        VCI_CloseDevice(VCI_USBCAN2,0);
        return false;
    }
    ROS_DEBUG("[CDeviceNetMaster][Init] VCI_StartCAN2 succeeded");
#endif
    m_bDeviceOpened = true;
    return true;
}

/*************************************************
Function: CUsbCan::SendCan
Description:　二次封装USBCAN发送库函数
Input: USB_CAN_OBJ *canFrames  can数据
       int          frameNum   要接受的can帧个数
Output: void
Others: void
**************************************************/
void CUsbCan::SendCan(USB_CAN_OBJ *canFrames, int frameNum)
{
    if(m_bDeviceOpened)
    {
        VCI_Transmit(VCI_USBCAN2, 0, 0, canFrames, (unsigned int)frameNum);
    }
}

/*************************************************
Function: CUsbCan::Receive
Description:　二次封装USBCAN接收库函数
Input: USB_CAN_OBJ *canFrames  can数据
       int          frameNum     要接受的can帧个数
Output: void
Others: void
**************************************************/
unsigned int CUsbCan::Receive(USB_CAN_OBJ *canFrames, unsigned bufLen, int nTimeout)
{
    unsigned int unRecvLen = 0;

    if(m_bDeviceOpened)
    {
        unRecvLen = VCI_Receive(VCI_USBCAN2, 0, 0, canFrames, bufLen, nTimeout/*ms*/);
    }

    return unRecvLen;
}

/*************************************************
Function: CUsbCan::Close
Description:　关闭USBCAN设备
Input: void
Output: void
Others: void
**************************************************/
void CUsbCan::Close()
{
    m_bDeviceOpened = false;
    Reset();
    VCI_CloseDevice(VCI_USBCAN2, 0);
}

/*************************************************
Function: CUsbCan::Close
Description:　关闭USBCAN设备
Input: void
Output: void
Others: void
**************************************************/
void CUsbCan::Reset()
{
    VCI_ClearBuffer(VCI_USBCAN2, 0, 0);//清空buffer。
    VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
#if USE_CAN2_RT
    VCI_ClearBuffer(VCI_USBCAN2, 0, 1);//清空buffer。
    VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
#endif
}

/*************************************************
Function: CUsbCan::SetCanFrame
Description:　设置can帧数据
Input: USB_CAN_OBJ       *canFrames  can数据
       UINT               ID         can帧中的ID
       BYTE               DataLen    can帧中数据的长度
       unsigned long long frameData  can帧中对应的数据
Output: void
Others: void
**************************************************/
void CUsbCan::SetCanFrame(USB_CAN_OBJ *canFrame, UINT ID, BYTE DataLen, unsigned long long frameData)
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
Function: CUsbCan::SetCanSpeed
Description:　设置can帧数据
Input: int nCanSpeed ,波特率大小
Output: void
Others: 其它非常规波特率，可以使用附带的波特率侦测工具进行侦测，并到得相应的波特率参数。或是使用USB_CAN TOOL安装目录下的波特率计算工具计算。
**************************************************/
void CUsbCan::SetCanSpeed(int nCanSpeed)
{
    switch (nCanSpeed)
    {
        case 5000:
            m_unBaud = 0xFFBF; break;
        case 10000:
            m_unBaud = 0x1C31; break;
        case 20000:
            m_unBaud = 0x1C18; break;
        case 40000:
            m_unBaud = 0xFF87; break;
        case 50000:
            m_unBaud = 0x1C09; break;
        case 80000:
            m_unBaud = 0Xff83; break;
        case 100000:
            m_unBaud = 0x1C04; break;
        case 125000:
            m_unBaud = 0x1C03; break;
        case 200000:
            m_unBaud = 0xFA81; break;
        case 250000:
            m_unBaud = 0x1C01; break;
        case 400000:
            m_unBaud = 0xFA80; break;
        case 500000:
            m_unBaud = 0x1C00; break;
        case 666000:
            m_unBaud = 0xB680; break;
        case 800000:
            m_unBaud = 0x1600; break;
        case 1000000:
            m_unBaud = 0x1400; break;
        default:
            m_unBaud = 0x1C00;//默认500K
            break;
    }
}

/*************************************************
Function: CUsbCan::PrintCanFrame
Description:　设置can帧数据
Input: USB_CAN_OBJ       *canFrames  can数据
       const char        *FuncName   调用者的函数名
Output: void
Others: void
**************************************************/
void CUsbCan::PrintCanFrame(USB_CAN_OBJ *canFrame, const char *FuncName)
{
    string sPrintStr;
    char buf[32];
    sPrintStr += "[";
    sPrintStr += FuncName;
    sPrintStr += "]";
    sPrintStr += "[printFrame]";

    struct timeval tv;
    char tmp[64];
    gettimeofday(&tv, nullptr);
    strftime(tmp, sizeof(tmp)-1, "TimeStamp:%H:%M:%S", localtime(&tv.tv_sec));
    sprintf(buf, "%s.%03d ", tmp, (int)(tv.tv_usec / 1000));
    sPrintStr += buf;

    sprintf(buf,"ID:0x%08X ", canFrame->ID);//ID
    sPrintStr += buf;

    if(canFrame->ExternFlag==0) sprintf(buf, "Standard ");//帧格式：标准帧
    if(canFrame->ExternFlag==1) sprintf(buf, "Extend   ");//帧格式：扩展帧
    sPrintStr += buf;

//    if(canFrame->RemoteFlag==0) sprintf(buf, "Data   ");//帧类型：数据帧
//    if(canFrame->RemoteFlag==1) sprintf(buf, "Remote ");//帧类型：远程帧
//    sPrintStr += buf;

    sprintf(buf, "Len:%d", canFrame->DataLen);//帧长度
    sPrintStr += buf;

    sPrintStr += " data:0x";	//数据
    for(int i = 0; i < canFrame->DataLen; i++)
    {
        sprintf(buf, " %02X", canFrame->Data[i]);
        sPrintStr += buf;
    }
//    printf(buf, " TimeStamp:0x%08X",canFrame->TimeStamp);//时间标识。
//    sPrintStr += buf;
//    printf("%s\n",sPrintStr.c_str());
    ROS_DEBUG("%s",sPrintStr.c_str());
}

/*************************************************
Function: CUsbCan::CanDataCpy
Description:　can帧拷贝函数
Input: USB_CAN_OBJ *desCanFrame 目标can帧
       USB_CAN_OBJ *souCanFrame 源can帧
Output: void
Others: void
**************************************************/
void CUsbCan::CanDataCpy(USB_CAN_OBJ *desCanFrame, USB_CAN_OBJ *souCanFrame)
{
    desCanFrame->ExternFlag = souCanFrame->ExternFlag;
    desCanFrame->RemoteFlag = souCanFrame->RemoteFlag;
    desCanFrame->SendType = souCanFrame->SendType;
    desCanFrame->TimeFlag = souCanFrame->TimeFlag;
    desCanFrame->TimeStamp = souCanFrame->TimeStamp;

    for(int i=0;i<3;i++)
        desCanFrame->Reserved[i] = souCanFrame->Reserved[i];

    desCanFrame->ID = souCanFrame->ID;
    desCanFrame->DataLen = souCanFrame->DataLen;
    for(int i=0; i<desCanFrame->DataLen ;i++)
    {
        desCanFrame->Data[i] = souCanFrame->Data[i];
    }
}

/*************************************************
Function: CUsbCan::CheckCanFrame
Description:　can帧拷贝函数
Input: USB_CAN_OBJ         *canFrame    待校验的CAN帧
       unsigned long long  frameData    与之校验的数据位字段
Output: void
Others: void
**************************************************/
bool CUsbCan::CheckCanFrame(const USB_CAN_OBJ *canFrame, unsigned long long frameData)
{
    unsigned int DataLen = canFrame->DataLen;

    for(int i=DataLen-1 ; i>=0 ; i--)
    {
        if(canFrame->Data[i] != u_char(frameData & 0xFF))
        {
            return false;
        }
        frameData >>= 8;
    }
    return true;
}

/*************************************************
Function: CUsbCan::CheckCanFrame
Description:　can帧拷贝函数
Input: USB_CAN_OBJ         *canFrame    待校验的CAN帧
       unsigned long long  frameData    与之校验的数据位字段
Output: void
Others: void
**************************************************/
int CUsbCan::CanFrameCmp(const USB_CAN_OBJ *desCan, const USB_CAN_OBJ *srcCan)
{
    if(desCan->ID != srcCan->ID || desCan->DataLen != srcCan->DataLen)
        return 1;

    for(int i=0 ; i< desCan->DataLen ; i++)
    {
        if(desCan->Data[i] != srcCan->Data[i])
        {
            return 1;
        }
    }
    return 0;
}
}