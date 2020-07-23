//
// Created by zhoujg on 19-1-24.
//

#ifndef PROJECT_USB_CAN_H
#define PROJECT_USB_CAN_H
#include "controlcan.h"
#include <iostream>
#include <thread>

#define USE_CAN2_RT 0

using namespace std;

namespace UsbCan
{
    typedef VCI_CAN_OBJ USB_CAN_OBJ;

    struct CanFrame : public VCI_CAN_OBJ
    {
        #define bExtend ExternFlag
        #define bRemote RemoteFlag
        #define u32Id ID
        #define u8Size DataLen
        #define szData Data
    };

    class CUsbCan
    {
    public:
        CUsbCan();

        ~CUsbCan();

        bool Init();

        unsigned int Receive(USB_CAN_OBJ *canFrames, unsigned bufLen, int unTimeout);

        void SetCanFrame(USB_CAN_OBJ *canFrame, UINT ID, BYTE DataLen, unsigned long long frameData);

        void SetCanSpeed(int nCanSpeed);

        void PrintCanFrame(USB_CAN_OBJ *canFrame, const char *FuncName = "");

        void CanDataCpy(USB_CAN_OBJ *desCanFrame, USB_CAN_OBJ *souCanFrame);

        void SendCan(USB_CAN_OBJ *canFrames, int frameNum);

        bool CheckCanFrame(const USB_CAN_OBJ *canFrame, unsigned long long frameData);

        int CanFrameCmp(const USB_CAN_OBJ *desCan, const USB_CAN_OBJ *srcCan);

        void Close();

        void Reset();

    private:

        VCI_INIT_CONFIG m_config;
        unsigned m_unBaud;
        bool m_bDeviceOpened;
    };
}
#endif //PROJECT_USB_CAN_H
