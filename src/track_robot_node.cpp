/*************************************************
Copyright: wootion
Author: ZhouJinGang
Date: 2019-2-25
Description: main
**************************************************/

#include <iostream>
#include <string>
#include <ctime>
#include <cmath>
#include <chrono>
#include <ros/ros.h>
#include "CTrackRobot.h"
#include <signal.h>

void SignalFunc(int sig)
{
    ROS_WARN("[SignalFunc]exit,sig=%d",sig);
    VCI_ClearBuffer(VCI_USBCAN2, 0, 0);//清空buffer。
    VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
#if USE_CAN2_RT
    VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN2通道。
#endif
    VCI_CloseDevice(VCI_USBCAN2, 0);
    exit(-1);
}

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "track_robot");

    signal(SIGINT, SignalFunc);
    signal(SIGABRT, SignalFunc);
    signal(SIGTERM, SignalFunc);
    signal(SIGPIPE, SignalFunc);
    signal(SIGKILL, SignalFunc);
    signal(SIGSEGV, SignalFunc);
    signal(SIGPWR, SignalFunc);

    CTrackRobot track_robot;

    ros::MultiThreadedSpinner spinner(6); // Use 6 threads
    spinner.spin(); // spin() will not return until the node has been shutdown
//    ros::spin();

    return 0;
}
