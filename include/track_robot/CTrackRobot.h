/*************************************************
Copyright: wootion
Author: ZhouJinGang
Date: 2019-2-25
Description: CTrackRobot
**************************************************/

#ifndef PROJECT_CTRACKROBOT_H
#define PROJECT_CTRACKROBOT_H

#include <iostream>
#include <ros/ros.h>
#include <string>
#include <deque>
#include <mutex>
#include <thread>
#include <queue>
#include <time.h>
#include <condition_variable>
#include <cmath>
#include <fcntl.h>
#include <math.h>
#include <fstream>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "custom_msgs/TrainRobotControl.h"
#include "custom_msgs/TrainJoyControlCmd.h"
#include "custom_msgs/TrainRobotPosition.h"
#include "custom_msgs/TrainRobotControlAck.h"
#include "custom_msgs/TrainRobotHeartBeat.h"
#include "custom_msgs/BatteryStatus.h"
#include "custom_msgs/CurrentPosition.h"
#include "custom_msgs/GeneralTopic.h"
#include "CGpioControl.h"
#include "CUsbCan.h"
#include "CMotor.h"
#include "sick_lms400.h"

//added by btrmg for adjust mileage 2020.01.07
#include "tinyxml2.h"
using namespace tinyxml2;
//added end
//added by btrmg for filter 20200413
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
//added end

using namespace UsbCan;
using namespace std;
using namespace asr_sick_lms400;

#define LASER_TRACK_CENTER_DISTANCE 0.5522 //激光原点距轨道中心距(m)
#define LASER_TRACK_SURFACE_DISTANCE 0.2306 //激光原点距轨面距离(m) 0.4066-0.176=0.2306
#define LASER_TRACK_POINT_DISTANCE 0.2837 //激光原点距轨面点到点距离(m) 根号((7175-5522)^2 + (2306)^2)
#define LASER_TRACK_POINT_ANGLE 54.366 //激光原点与轨面点连线与水平面夹角(°)
#define HALF_OF_TRACK_DISTANCE (1.435/2) //轨内距离的一半长度(m)
//#define LASER_GROUND_DISTANCE 0.2772 //激光原点距地面距离(m) 轨高176mm + 0.2772
#define LASER_ANGLE_RANGE 70.0 //激光角度量程(°)
#define LASER_EXCISION_ANGLE 5 //激光从下部切除的角度
#define LASER_ANGLE_INCREMENT (70.0/840) //激光角度间隔(°)
#define SPEED_INCREMENT 0.1 //速度增量间隔值(m/s)

#define END_DIS_MODIFICATION 0.52     //用于速度模式下即将到达目标点的匀减速里程计算参数
#define END_DIS_AMEND 0.25     //EndDis修正值
#define LIMIT_LOCATION_RANGE_PARAM 0.00     //侵限点定位时的前后范围值
#define LASER_MAX_DATA_LEN 841     //激光单次测量所返回的最大数据个数
#define CURRENT_DATA_TOTAL_NUM 5    //取当前点位置时，用于求平均数的数据的数量。

typedef unsigned long long ULL;


typedef enum _tagRobotDirection_
{
    FORWARD = 1,
    BACKWARD,
}RobotDirection;

typedef enum _tagAutoRunStatus_
{
    INITIAL,//初始状态
    LOCATION_DONE//run forward 或者 run backward 后到达位置点
}AutoRunStatus;

typedef enum _tagControlMOde_
{
	JOY,
	TASK,
	ALARM,
	IDLE
}ControlMOde;

typedef enum _tagRobotStatus_
{
    RUN_FORWARD,
    RUN_BACKWARD,
    STOP,
    START,
    RETURN,
    TASK_DONE,
    RELOCATION_DONE,
    RETURN_DONE,
    ERROR
}RobotStatus;

const std::vector<std::string> vsRobotStatus
{
    "forward",
    "backward",
    "stop",
    "start",
    "return",
    "task_done",
    "relocation_done",
    "return_done",
    "error",
};

typedef enum _tagMoveType
{
    ONLY_MOVE,
    UPDATE_ORIGIN_MOVE_PUB,
    MOVE_PUB,
    SINGLE_MOVE,
    UPDATE_ORIGIN,
    UPDATE_ORIGIN_PLUS,
    UPDATE_ORIGIN_SUB,
}MoveType;

typedef enum _tagGpioCtrlWords
{
    LOW,
    HIGH,
    LOGIC_ON,
    LOGIC_OFF,
    R_LIGHT_ON,
    R_LIGHT_OFF,
    G_LIGHT_ON,
    G_LIGHT_OFF,
    B_LIGHT_ON,
    B_LIGHT_OFF,
    POWER_ON,
    POWER_OFF,
    GET_EMERGENCY_STOP_BUTTON_STATUS
}GpioCtrlWords;

typedef struct _tagMotorPosition
{
    int nMotorOnePos;
    int nMotorTwoPos;
} MotorPosition, *PMotorPosition;

typedef struct _tagScanBuf
{
    float fRanges[LASER_MAX_DATA_LEN];
    double x;
    ULL ullRangeSize;
} sScanBuf, *PScanBuf;

typedef struct _tagPositionData
{
    double x;
    float y[LASER_MAX_DATA_LEN];
    float z[LASER_MAX_DATA_LEN];
    float laser_dis[LASER_MAX_DATA_LEN];
    float laser_angle[LASER_MAX_DATA_LEN];
} sPositionData, *PPositionData;

class CTrackRobot
{
public:
    CTrackRobot();
    ~CTrackRobot();
    void CANReceiveThreadFunc();
#if USE_CAN2_RT
    void CAN2ReceiveThreadFunc();
#endif
    void CANManageThreadFunc();
    void CANSendThreadFunc();
    void CheckTaskStatus();
    void UpdateMotorInfoTimerFunc();
    void ManageScanDataThreadFunc();
    void CheckJoyStatus();
    void CheckCanStatus();
    void CheckEStopStatus();
    void HeartBeatThreadFunc();
    void MonitorStatusThreadFunc();

    int MotorServoInit();
    void AccelerationOrDeceleration();
    void CalcRobotMileage();
    int SetRobotAcceleration(double dAcceleration);
    int SetRobotDeceleration(double dDeceleration);
    void ForwardMotion();
    void BackwardMotion();
    void QuickStop();
    void StopMotion(bool bJoy = false);
    void MotorPowerOff();
    void ClearWarn();
    void SendGetMotorInfoCmd();
    void MotorSpeedUp();
    void MotorSpeedDown();
    void SetMotorSpeed(double dSpeed);
    void SendCmdOfGetMotorTemp();
    void SendCmdOfGetMotorErrorCode();
    void SendHeartBeatCanFrame();
    void SendCmdOfGetBatteryInfo();
    void SetNewPos(double dDistance);
    void SendCanFrame(unsigned ID, unsigned char DataLen, ULL frameData);
    int SendCanFrame(CanFrame *sendCanData, int nCount, CanFrame *checkCanData = nullptr, bool bIsCheck = false);
    void PublishOdom();
    void PublishStatus(int nStatus);
    void PublishStatus(string &sStatus);
    int TransPosBufData(sPositionData &desPosData, const sScanBuf &srcBufData, bool bAbsolute=false);
    void OutputPosData(sPositionData &desPosData, int nDataLen);
    void UpdateOriginPosition();
    bool WaitForMotionStop(int nTimeOutLen);
    int RestartMotorServo();
    int GpioControl(int nCtrlWord);

    void CommandCallBack(const custom_msgs::TrainRobotControl::ConstPtr &Command);
    void OdomCallBack(const nav_msgs::Odometry::ConstPtr &OdomMsg);
    bool BatteryServiceFunc(custom_msgs::BatteryStatus::Request &Req, custom_msgs::BatteryStatus::Response &Resp);
    bool CurrentPosServiceFunc(custom_msgs::CurrentPosition::Request &Req, custom_msgs::CurrentPosition::Response &Resp);
    void ImuCallBack(const sensor_msgs::Imu::ConstPtr &ImuData);
    void ScanThreadFunc();
    //added by btrmg for adjust mileage 2020.01.07
    int updata_XML(string paramName, string paramValue);
    //added end
    //added by btrmg for filter point data 2020.04.13
    void FilterPoints(vector<float> in_y,vector<float > in_z,vector<double > &out_y,vector<double > &out_z);
    //added end

private:
    UsbCan::CUsbCan UsbCan;
    MotorPosition m_DynamicOrigin;
    MotorPosition m_CurrentMotorPos;
    timespec m_tTimeOfLastJoyCmd;
    timespec m_tMotorOneLastRespTime;
    timespec m_tMotorTwoLastRespTime;

    int m_nCanSpeed;
    int m_nPrintCanRTX;
    int m_nUseImuOdom;
    int m_nMode;
    int m_nDirection;
	int m_nControlMode;
	int m_nJoyCtrlTimeoutMSec;
    int m_nBatteryCurrent;
    int m_nBatteryCharge;
    int m_nSdoCheckCount;
    int m_nLightStatus;
    int m_nMotorOneTemp;
    int m_nMotorTwoTemp;
    int m_nBatteryTemp;
    int m_nImuBufSize;
    int m_nRecordLaser;
    int m_n1stFilterNum;
    int m_n2ndFilterNum;
    int m_nRobotOriginPulseOne;
    int m_nRobotOriginPulseTwo;
    int m_nRobotCurrentPulseOne;
    int m_nRobotCurrentPulseTwo;

    uint8_t m_nBatteryPower;
    uint8_t m_nBatteryStatus;
    uint16_t m_nBatteryVoltage;

    bool m_bIsMotorInit;
    bool m_bMotorRunning;
    bool m_bIsMotorPause;
    bool m_bIsInitializing;
    bool m_bCheckAlarmData;
    bool m_bGetAxesData;
    bool m_bForwardRelocation;
    bool m_bCheckSdo;
    bool m_bIsEStopButtonPressed;
	bool m_bIsUpdateOrigin;//用于融合节点求里程时的原点更新
	bool m_bPubPosData;
	bool m_bManagerUpdateOrigin;//用于相机校准求里程时的原点更新
	bool m_bSingleMove;

    double m_dDistance;
    double m_dCurrentOdom;
    double m_dOdom;
    double m_dDisTemp;
    double m_dTargetDis;
    double m_dTimerCycle;
    double m_dEndDis;
    double m_dCurrentSpeed1;
    double m_dCurrentSpeed;
    double m_dTargetSpeed;
    double m_dTargetSpeedTemp;
    double m_dInitialAngle;
    double m_dPitchAngle;
    double m_dPvAcceleration;
    double m_dScanTitle;
    double m_dRealScanTitle;
    double m_dRealDis;
    double m_d1stFilterRadius;
    double m_d2ndFilterRadius;
    double m_dAdjustLimit;

    std::string m_sMode;
    std::string m_sSubManagerCmdTopic;
    std::string m_sSubJoyTopic;
    std::string m_sSubScanTopic;
    std::string m_sSubImuTopic;
    std::string m_sSubOdomTopic;
    std::string m_sPubPositionTopic;
    std::string m_sPubStatusTopic;
    std::string m_sMoveAckTopic;
    std::string m_sPubOdomTopic;
    std::string m_sHeartBeatTopic;
    std::string m_sPrintLevel;
    std::string m_sMotorOneStatus;
    std::string m_sMotorTwoStatus;

    std::thread *m_pCANReceiveThread;
    std::thread *m_pCAN2ReceiveThread;
    std::thread *m_pCANManageThread;
    std::thread *m_pCANSendThread;
    std::thread *m_pControlThread;
    std::thread *m_pAutoRunThread;
    std::thread *m_pPositionFilterThread;
    std::thread *m_pCheckStatusThread;
    std::thread *m_pHeartBeatThread;
    std::thread *m_pScanThread;

    std::mutex m_CurrentSpeedMutex;
    std::mutex m_CanRXMutex;
    std::mutex m_ScanSrcBufMutex;
    std::mutex m_ScanManBufMutex;
    std::mutex m_CurrentPosScanBufMutex;
    std::mutex m_CanTXMutex;
    std::mutex m_ButteryStatusMutex;
    std::mutex m_JoyTimeOutCheckMutex;
    std::mutex m_SdoCheckMutex;

    std::deque<sScanBuf> m_dScanSrcBufDeque;
    std::deque<sScanBuf> m_dCurrentPosScanBuf;
    std::deque<sScanBuf> m_dScanManBufDeque;
    std::deque<CanFrame> m_dCanTXDeque;
    std::deque<CanFrame> m_dCanRXDeque;
    std::deque<double> m_dImuPitchDeque;

    std::condition_variable m_CanTXCondition;
    std::condition_variable m_CanRXCondition;
    std::condition_variable m_ScanBufCondition;
    std::condition_variable m_QrySpeedCondition;
    std::condition_variable m_QryPositionCondition;

    ros::Subscriber m_CmdManagerSubscriber;
    ros::Subscriber m_JoystickSubscriber;
    ros::Subscriber m_ScanSubscriber;
    ros::Subscriber m_ImuSubscriber;
    ros::Subscriber m_OdomSubscriber;

    ros::Publisher m_PositionPublisher;
    ros::Publisher m_HeartBeatPublisher;
    ros::Publisher m_OdomPublisher;
    ros::Publisher m_StatusPublisher;
    ros::Publisher m_MoveAckPublisher;

    ros::ServiceServer m_BatteryService;
    ros::ServiceServer m_CurrentPosService;

    ros::Timer m_DisplacementTimer;

    CMotor m_MotorOne;
    CMotor m_MotorTwo;

    CanFrame *m_SdoCheckBuf;
};

#endif //PROJECT_CTRACKROBOT_H
