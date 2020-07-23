/*************************************************
Copyright: wootion
Author: ZhouJinGang
Date: 2019-2-25
Description: CTrackRobot
**************************************************/

#include "CTrackRobot.h"

double actual_tire_diameter;
double forward_diameter;
double back_diameter;

CTrackRobot::CTrackRobot():
    m_nDirection(FORWARD),
    m_nJoyCtrlTimeoutMSec(300),
    m_nBatteryPower(0),
    m_nBatteryStatus(0),
    m_nBatteryVoltage(0),
    m_nSdoCheckCount(0),
    m_nLightStatus(0),
    m_nControlMode(IDLE),
    m_nRobotOriginPulseOne(0),
	m_nRobotOriginPulseTwo(0),
    m_bMotorRunning(false),
    m_bIsMotorInit(false),
    m_bIsMotorPause(false),
    m_bIsInitializing(false),
    m_bCheckAlarmData(false),
    m_bGetAxesData(false),
    m_bForwardRelocation(true),
    m_bCheckSdo(true),
    m_bIsEStopButtonPressed(true),
    m_bIsUpdateOrigin(false),
    m_bPubPosData(true),
    m_bManagerUpdateOrigin(true),
    m_bSingleMove(false),
    m_dDistance(0),
    m_dOdom(0),
    m_dCurrentSpeed(0),
    m_dTargetSpeedTemp(0),
    m_dPitchAngle(0.0),
    m_nMotorOneTemp(0),
    m_nMotorTwoTemp(0),
    m_sMotorOneStatus("normal"),
    m_sMotorTwoStatus("normal")
{
    ros::NodeHandle PublicNodeHandle;
    ros::NodeHandle PrivateNodeHandle("~");

    PrivateNodeHandle.param("can_speed", m_nCanSpeed, 500000);
    PrivateNodeHandle.param("print_can", m_nPrintCanRTX, 0);
    PrivateNodeHandle.param("use_imu_odom", m_nUseImuOdom, 0);
    PrivateNodeHandle.param("sport_mode", m_sMode, std::string("PP"));
    PrivateNodeHandle.param("motor_speed", m_dTargetSpeed, 0.0);
    PrivateNodeHandle.param("pv_a", m_dPvAcceleration, 0.2);
    PrivateNodeHandle.param("target_dis", m_dTargetDis, 10.0);
    PrivateNodeHandle.param("scan_title", m_dScanTitle, 20.0);
    PrivateNodeHandle.param("timer_cycle", m_dTimerCycle, 10.0);
    PrivateNodeHandle.param("print_level", m_sPrintLevel, std::string("debug"));
    PrivateNodeHandle.param("imu_size", m_nImuBufSize, 2);
    //added by btrmg for adjust mileage 2020.01.07
    PrivateNodeHandle.param("forward_diameter",forward_diameter,143.175);
    PrivateNodeHandle.param("back_diameter",back_diameter,143.175);
    //added end
    PrivateNodeHandle.param("record_laser", m_nRecordLaser, 1);
    PrivateNodeHandle.param("first_filter_radius", m_d1stFilterRadius, 0.1);
    PrivateNodeHandle.param("first_filter_num", m_n1stFilterNum, 30);
    PrivateNodeHandle.param("second_filter_radius", m_d2ndFilterRadius, 0.01);
    PrivateNodeHandle.param("second_filter_num", m_n2ndFilterNum, 3);
    PrivateNodeHandle.param("adjust_limit", m_dAdjustLimit, 2.0);

    PublicNodeHandle.param("sub_move_cmd_topic", m_sSubManagerCmdTopic, std::string("train_move_cmd"));
    PublicNodeHandle.param("sub_joy_topic", m_sSubJoyTopic, std::string("train_joy"));
    PublicNodeHandle.param("sub_scan_topic", m_sSubScanTopic, std::string("scan"));
    PublicNodeHandle.param("sub_scan_topic", m_sSubImuTopic, std::string("imu0"));
    PublicNodeHandle.param("robot_odom", m_sSubOdomTopic, std::string("odom"));
    PublicNodeHandle.param("pub_pos_topic", m_sPubPositionTopic, std::string("train_robot_position"));
    PublicNodeHandle.param("pub_task_status_topic", m_sPubStatusTopic, std::string("train_task_status"));
    PublicNodeHandle.param("pub_move_ack_topic", m_sMoveAckTopic, std::string("train_move_ack"));
    PublicNodeHandle.param("track_odom", m_sPubOdomTopic, std::string("robot_odom"));
    PublicNodeHandle.param("pub_heart_beat_topic", m_sHeartBeatTopic, std::string("train_motor_heart_beat"));

    // Se the logging level manually to Debug, Info, Warn, Error
    ros::console::levels::Level printLevel = ros::console::levels::Info;
    if(m_sPrintLevel == "debug")
        printLevel = ros::console::levels::Debug;
    else if(m_sPrintLevel == "warn")
        printLevel = ros::console::levels::Warn;
    else if(m_sPrintLevel == "error")
        printLevel = ros::console::levels::Error;

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, printLevel);

    ROS_INFO("usb can parameters:");

    ROS_INFO("[ros param] can_speed:%d", m_nCanSpeed);
    ROS_INFO("[ros param] print_can_flag:%d", m_nPrintCanRTX);
    ROS_INFO("[ros param] use_imu_odom:%d", m_nUseImuOdom);
    ROS_INFO("----------------------------------");
    ROS_INFO("motor control parameters:");
    ROS_INFO("[ros param] sport_mode:%s", m_sMode.c_str());
    ROS_INFO("[ros param] motor_speed:%f", m_dTargetSpeed);
    ROS_INFO("[ros param] pv_a:%f", m_dPvAcceleration);
    ROS_INFO("[ros param] target_dis:%f", m_dTargetDis);
    ROS_INFO("[ros param] scan_title:%f", m_dScanTitle);
    ROS_INFO("[ros param] timer_cycle:%.1f", m_dTimerCycle);
    ROS_INFO("[ros param] print_level:%s", m_sPrintLevel.c_str());
    ROS_INFO("[ros param] imu_size:%d", m_nImuBufSize);
    //added by btrmg for adjust mileage 2020.01.07
    ROS_INFO("[ros param] forward_diameter:%f", forward_diameter);
    ROS_INFO("[ros param] back_diameter:%f", back_diameter);
    //added end
    ROS_INFO("[ros param] record_laser:%d", m_nRecordLaser);
    ROS_INFO("[ros param] first_filter_radius:%f", m_d1stFilterRadius);
    ROS_INFO("[ros param] first_filter_num:%d", m_n1stFilterNum);
    ROS_INFO("[ros param] second_filter_radius:%f", m_d2ndFilterRadius);
    ROS_INFO("[ros param] second_filter_num:%d", m_n2ndFilterNum);

    ROS_INFO("----------------------------------");
    ROS_INFO("ros topics:");
    ROS_INFO("[ros param] sub_manager_cmd_topic:%s", m_sSubManagerCmdTopic.c_str());
    ROS_INFO("[ros param] sub_joy_topic:%s", m_sSubJoyTopic.c_str());
    ROS_INFO("[ros param] sub_scan_topic:%s", m_sSubScanTopic.c_str());
    ROS_INFO("[ros param] sub_imu_topic:%s", m_sSubImuTopic.c_str());
    ROS_INFO("[ros param] pub_stat_topic:%s", m_sPubPositionTopic.c_str());
    ROS_INFO("[ros param] pub_status_topic:%s", m_sPubStatusTopic.c_str());
    ROS_INFO("[ros param] pub_manager_status_topic:%s", m_sMoveAckTopic.c_str());
    ROS_INFO("[ros param] pub_heart_beat_topic:%s", m_sHeartBeatTopic.c_str());

    ROS_INFO("----------------------------------");

    if(m_sMode == "PP" && 1 != m_nUseImuOdom)
    {
        m_nMode = PP;
    }
    else
    {
        m_nMode = PV;
    }

    actual_tire_diameter = forward_diameter;

    m_dInitialAngle = 180.0-m_dScanTitle-(90.0-LASER_ANGLE_RANGE/2)-LASER_EXCISION_ANGLE;

    UsbCan.SetCanSpeed(m_nCanSpeed);

    if (!UsbCan.Init())
    {
        ROS_ERROR("init usbcan failed");
        exit(-1);
    }

    try
    {
        m_pCANReceiveThread = new std::thread(std::bind(&CTrackRobot::CANReceiveThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CTrackRobot::CTrackRobot] malloc CAN receive thread failed, %s", exception.what());
        exit(-1);
    }

#if USE_CAN2_RT
    try
    {
        m_pCAN2ReceiveThread = new std::thread(std::bind(&CTrackRobot::CAN2ReceiveThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CTrackRobot::CTrackRobot] malloc CAN2 receive thread failed, %s", exception.what());
        exit(-1);
    }
#endif

    try
    {
        m_pCANManageThread = new std::thread(std::bind(&CTrackRobot::CANManageThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CTrackRobot::CTrackRobot] malloc CAN receive thread failed, %s", exception.what());
        exit(-1);
    }

    try
    {
        m_pCANSendThread = new std::thread(std::bind(&CTrackRobot::CANSendThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CTrackRobot::CTrackRobot] malloc CAN Send thread failed, %s", exception.what());
        exit(-1);
    }

    try
    {
        m_pPositionFilterThread = new std::thread(std::bind(&CTrackRobot::ManageScanDataThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CTrackRobot::CTrackRobot] malloc filter Thread failed, %s", exception.what());
        exit(-1);
    }

    this_thread::sleep_for(std::chrono::milliseconds(500));

    m_ImuSubscriber = PublicNodeHandle.subscribe<sensor_msgs::Imu>(m_sSubImuTopic, 10, boost::bind(&CTrackRobot::ImuCallBack, this, _1));
    m_CmdManagerSubscriber = PublicNodeHandle.subscribe<custom_msgs::TrainRobotControl>(m_sSubManagerCmdTopic, 1, boost::bind(
            &CTrackRobot::CommandCallBack, this, _1));

    if(1 == m_nUseImuOdom)
    {
        m_OdomSubscriber = PublicNodeHandle.subscribe<nav_msgs::Odometry>(m_sSubOdomTopic, 1, boost::bind(&CTrackRobot::OdomCallBack, this, _1));
    }

    m_PositionPublisher = PublicNodeHandle.advertise<custom_msgs::TrainRobotPosition>(m_sPubPositionTopic, 10);
    m_StatusPublisher = PublicNodeHandle.advertise<custom_msgs::GeneralTopic>(m_sPubStatusTopic, 10);
    m_MoveAckPublisher = PublicNodeHandle.advertise<custom_msgs::TrainRobotControlAck>(m_sMoveAckTopic, 10);
    m_HeartBeatPublisher = PublicNodeHandle.advertise<custom_msgs::TrainRobotHeartBeat>(m_sHeartBeatTopic, 10);
    m_OdomPublisher = PublicNodeHandle.advertise<nav_msgs::Odometry>(m_sPubOdomTopic, 10);

    m_BatteryService = PublicNodeHandle.advertiseService("battery_status", &CTrackRobot::BatteryServiceFunc, this);
    m_CurrentPosService = PublicNodeHandle.advertiseService("current_position", &CTrackRobot::CurrentPosServiceFunc, this);
    m_DisplacementTimer = PublicNodeHandle.createTimer(ros::Duration(m_dTimerCycle/1000.0), boost::bind(&CTrackRobot::UpdateMotorInfoTimerFunc, this));

    this_thread::sleep_for(std::chrono::milliseconds(500));

    if(!m_bIsInitializing)
    {
        MotorServoInit();
    }

    try
    {
        m_pHeartBeatThread = new std::thread(std::bind(&CTrackRobot::HeartBeatThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CTrackRobot::CTrackRobot] malloc heart beat Thread failed, %s", exception.what());
        exit(-1);
    }

    try
    {
        m_pScanThread = new std::thread(std::bind(&CTrackRobot::ScanThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CTrackRobot::CTrackRobot] malloc scan Thread failed, %s", exception.what());
        exit(-1);
    }

    try
    {
        m_pScanThread = new std::thread(std::bind(&CTrackRobot::MonitorStatusThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CTrackRobot::CTrackRobot] malloc monitor Thread failed, %s", exception.what());
        exit(-1);
    }
}

/*************************************************
Function: CTrackRobot::CANReceiveThreadFunc
Description:　USBCAN设备数据接收线程功能函数
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::CANReceiveThreadFunc()
{
    unsigned int unRecvLen;
    CanFrame rec[2500];
    int nRetryTimes = 0;
    while (ros::ok())
    {
        try
        {
//            if((unRecvLen=UsbCan.Receive(rec, 2500, 0))>0)
            if((unRecvLen = VCI_Receive(VCI_USBCAN2, 0, 0, rec, 2500, 0)) > 0)
            {
                if(unRecvLen > 2500)
                {
                    if(nRetryTimes < 3)
                    {
                        UsbCan.Reset();
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                        ROS_WARN("[CANReceiveThreadFunc] recLen = %d",unRecvLen);
                        nRetryTimes ++;
                        continue;
                    }
                    else
                    {
                        UsbCan.Close();
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                        exit(-1);
                    }

                }
                nRetryTimes = 0;
                std::unique_lock<std::mutex> lock(m_CanRXMutex);
                for(int i=0; i<unRecvLen; i++)
                {
                    if(rec[i].DataLen <=0 || rec[i].DataLen >8)
                        continue;

                    if(m_nPrintCanRTX)
                        UsbCan.PrintCanFrame(&rec[i], __FUNCTION__);
                    m_dCanRXDeque.push_back(rec[i]);
                }
                lock.unlock();
                m_CanRXCondition.notify_one();
            }
        }
        catch(...)
        {
            UsbCan.Reset();
            ROS_ERROR("[CANReceiveThreadFunc] catch can receive func error");
        }
    }
}

#if USE_CAN2_RT
/*************************************************
Function: CDeviceNetMaster::CAN2ReceiveThreadFunc
Description: 用于记录总线上的交互报文数据
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::CAN2ReceiveThreadFunc()
{
    ROS_DEBUG("[CAN2ReceiveThreadFunc] running");
    int i;
    ULL recLen;
    CanFrame rec[3000];
    string sDataFileName = m_sDataFilePath;

    sDataFileName += "servo/";
    sDataFileName += "ServoInit";
    CFileRW ServoInitFileRw;
    time_t t = time(nullptr);
    char cTmp[64];
    strftime( cTmp, sizeof(cTmp), "%Y%m%d-%H%M%S",localtime(&t));
    sDataFileName += cTmp;
    sDataFileName += ".txt";

    ServoInitFileRw.OpenFile(sDataFileName, std::string("a+"));

    ROS_DEBUG("[CreateNewRecordFile] open file:%s",sDataFileName.c_str());
    while (ros::ok())
    {
        if(!m_bIsMotorInit)
        {
            if((recLen=VCI_Receive(VCI_USBCAN2, 0, 1, rec, 3000, 100/*ms*/))>0)
            {
                for(i=0; i<recLen; i++)
                {
                    if(rec[i].DataLen <=0 || rec[i].DataLen >8)
                        continue;

                    string sPrintStr;
                    char buf[32];

                    timeval tv;
                    char cTimeTmp[64];

                    gettimeofday(&tv, nullptr);
                    strftime(cTimeTmp, sizeof(cTimeTmp)-1, "TimeStamp:%Y/%m/%d-%H:%M:%S", localtime(&tv.tv_sec));
                    sprintf(buf, "%s.%03d ", cTimeTmp, (int)(tv.tv_usec / 1000));
                    sPrintStr += buf;

                    sprintf(buf,"ID:0x%08X ", rec[i].ID);//ID
                    sPrintStr += buf;

                    if(rec[i].ExternFlag==0) sprintf(buf, "Standard ");//帧格式：标准帧
                    if(rec[i].ExternFlag==1) sprintf(buf, "Extend   ");//帧格式：扩展帧
                    sPrintStr += buf;

                    if(rec[i].RemoteFlag==0) sprintf(buf, "Data   ");//帧类型：数据帧
                    if(rec[i].RemoteFlag==1) sprintf(buf, "Remote ");//帧类型：远程帧
                    sPrintStr += buf;

                    sprintf(buf, "Len:%d", rec[i].DataLen);//帧长度
                    sPrintStr += buf;

                    sPrintStr += " data:0x";	//数据
                    for(int j = 0; j < rec[i].DataLen; j++)
                    {
                        sprintf(buf, " %02X", rec[i].Data[j]);
                        sPrintStr += buf;
                    }

                    sPrintStr += "\n";
                    ServoInitFileRw.Output(sPrintStr);
                }
            }
        }
        else
        {
            ServoInitFileRw.CloseFile();
            break;
        }

    }
}
#endif

/*************************************************
Function: CTrackRobot::CANManageThreadFunc
Description:　USBCAN设备数据处理线程功能函数
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::CANManageThreadFunc()
{
    ROS_INFO("[CANManageThreadFunc] start");

    CanFrame canTemp;
    std::deque<CanFrame> vCanFrameDeque;
    unsigned long ulSize;
    int nMotorOnePrePulse = 0;
    int nMotorTwoPrePulse = 0;

    while(ros::ok())
    {
        std::unique_lock<std::mutex> lock(m_CanRXMutex);
        while(m_dCanRXDeque.empty())
        {
            m_CanRXCondition.wait(lock);
        }

        vCanFrameDeque.assign(m_dCanRXDeque.begin(), m_dCanRXDeque.end());
        m_dCanRXDeque.clear();
        lock.unlock();

        ulSize = vCanFrameDeque.size();
        for (int i = 0; i < ulSize; i++)
        {
            canTemp = vCanFrameDeque[i];
            //PDO Motor INFO
            if(canTemp.u32Id == m_MotorOne.m_unPdoRxCobId)
            {
                timespec_get(&m_tMotorOneLastRespTime, TIME_UTC);
                m_MotorOne.UpdateMotorStatus(canTemp);

                m_CurrentMotorPos.nMotorOnePos = m_MotorOne.m_MotorStatus.nCurrentPos;
                m_dCurrentSpeed1 = m_MotorOne.m_MotorStatus.dActual_speed;
                m_bMotorRunning = (abs(m_dCurrentSpeed) > 0.001);
                if(m_dCurrentSpeed >= 0)
                {
                    m_nRobotCurrentPulseOne += m_CurrentMotorPos.nMotorOnePos - nMotorOnePrePulse;
                }
                else
                {
                    m_nRobotCurrentPulseOne += int((m_CurrentMotorPos.nMotorOnePos - nMotorOnePrePulse)*(back_diameter/forward_diameter));
                }

                nMotorOnePrePulse = m_CurrentMotorPos.nMotorOnePos;
            }
            else if(canTemp.u32Id == m_MotorTwo.m_unPdoRxCobId)
            {
                timespec_get(&m_tMotorTwoLastRespTime, TIME_UTC);
                m_MotorTwo.UpdateMotorStatus(canTemp);

                m_CurrentMotorPos.nMotorTwoPos = m_MotorTwo.m_MotorStatus.nCurrentPos;
                m_dCurrentSpeed = (m_MotorTwo.m_MotorStatus.dActual_speed + m_dCurrentSpeed1)/2;
                m_bMotorRunning = (abs(m_dCurrentSpeed) > 0.001);

                //位置模式下设置,跟随电机速度设置
                if(m_nMode == PP && !m_bSingleMove)
                {
                    CanFrame SpeedCanData;
                    SpeedCanData.ID = m_MotorOne.m_unPdoTxCobId;
                    SpeedCanData.RemoteFlag = 0;
                    SpeedCanData.ExternFlag = 0;
                    SpeedCanData.SendType = 0;
                    SpeedCanData.DataLen = 2;
                    SpeedCanData.Data[0] = canTemp.Data[0];
                    SpeedCanData.Data[1] = canTemp.Data[1];

                    SendCanFrame(&SpeedCanData, 1);
                }

                if(1 != m_nUseImuOdom)
                {
                    if(m_bMotorRunning)
                    {
                        CalcRobotMileage();
                    }
                }

                if(m_dCurrentSpeed >= 0)
                {
                    m_nRobotCurrentPulseTwo += m_CurrentMotorPos.nMotorTwoPos - nMotorTwoPrePulse;
                }
                else
                {
                    m_nRobotCurrentPulseTwo += int((m_CurrentMotorPos.nMotorTwoPos - nMotorTwoPrePulse)*(back_diameter/forward_diameter));
                }
                nMotorTwoPrePulse = m_CurrentMotorPos.nMotorTwoPos;
            }

            //Motor Sdo Check
            else if(canTemp.u32Id == m_MotorOne.m_unSdoRxCobId || canTemp.u32Id == m_MotorTwo.m_unSdoRxCobId)
            {
                if(m_bCheckSdo)
                {
                    std::unique_lock<std::mutex> Lock(m_SdoCheckMutex);
                    if(UsbCan.CanFrameCmp(&m_SdoCheckBuf[m_nSdoCheckCount],&canTemp) != 0)
                    {
                        UsbCan.PrintCanFrame(&m_SdoCheckBuf[m_nSdoCheckCount],"SdoCheckBuf");
                        UsbCan.PrintCanFrame(&canTemp,"recvCanData");
                        m_bCheckSdo = false;
                    }
                    m_nSdoCheckCount++;
                }
                else if(canTemp.u32Id == m_MotorOne.m_unSdoRxCobId)
                {
                    m_MotorOne.ManageSdoInfo(canTemp);
                    m_sMotorOneStatus = m_MotorOne.m_MotorStatus.sErr;
                    if(m_sMotorOneStatus != "normal" && m_bIsMotorInit)
                    {
                        ROS_ERROR("[CANManageThreadFunc] Motor1 servo error");
                        m_dTargetSpeed = 0;
                        m_bIsMotorInit = false;

                        if(GpioControl(G_LIGHT_OFF) == -1)
                            ROS_ERROR("[MotorServoInit] turn off green light failed");
                        if(GpioControl(R_LIGHT_ON) == -1)
                            ROS_ERROR("[MotorServoInit] turn on red light failed");
                    }
                }
                else if(canTemp.u32Id == m_MotorTwo.m_unSdoRxCobId)
                {
                    m_MotorTwo.ManageSdoInfo(canTemp);
                    m_sMotorTwoStatus = m_MotorTwo.m_MotorStatus.sErr;

                    if(m_sMotorTwoStatus != "normal" && m_bIsMotorInit)
                    {
                        ROS_ERROR("[CANManageThreadFunc] Motor2 servo error");
                        m_dTargetSpeed = 0;
                        m_bIsMotorInit = false;

                        if(GpioControl(G_LIGHT_OFF) == -1)
                            ROS_ERROR("[MotorServoInit] turn off green light failed");
                        if(GpioControl(R_LIGHT_ON) == -1)
                            ROS_ERROR("[MotorServoInit] turn on red light failed");
                    }
                }
            }

//字节1 电池电压(单位0.1V,400->40V);字节2 电池电压(单位0.1V,400->40V);字节3 电池电流(单位0.1A,200->20A);字节4 电池电流(单位0.1A,200->20A)
//字节5 电量信息(SOC,0-100);字节6	电池温度(65表示25℃,负偏40);字节7 充电状态：1正在充电，0没有充电;字节8	电池组串数
            else if(canTemp.u32Id == 0xBB)
            {
                m_nBatteryVoltage = canTemp.szData[0];
                m_nBatteryVoltage <<= 8;
                m_nBatteryVoltage += canTemp.szData[1];

                m_nBatteryCurrent = canTemp.szData[2];
                m_nBatteryCurrent <<= 8;
                m_nBatteryCurrent += canTemp.szData[3];

                m_nBatteryPower = canTemp.szData[4];
                m_nBatteryTemp = canTemp.szData[5] - 40;
                m_nBatteryCharge = canTemp.szData[6];
            }
//#电池状态：bit0：欠压 bit1：过压 bit2：过流 bit3：低温 bit4：高温 bit5：充电中 bit6：放电中 bit7：充电fet损坏
            else if(canTemp.u32Id == 0xBE)
            {
                uint8_t nBatteryStatus;
                nBatteryStatus = 0x00;
                nBatteryStatus |= (canTemp.szData[1] & 0xAA)? 0x01:0x00;//欠压
                nBatteryStatus |= (canTemp.szData[1] & 0x55)? 0x02:0x00;//过压
                nBatteryStatus |= (canTemp.szData[3] & 0x34)? 0x04:0x00;//过流

                nBatteryStatus |= (canTemp.szData[4] & 0xAA)? 0x08:0x00;
                nBatteryStatus |= (canTemp.szData[5] & 0xAA)? 0x08:0x00;//低温

                nBatteryStatus |= (canTemp.szData[4] & 0x55)? 0x10:0x00;
                nBatteryStatus |= (canTemp.szData[5] & 0x55)? 0x10:0x00;//高温

//                nBatteryStatus |= (canTemp.szData[3] & 0x01)? 0x20:0x00;//充电状态
//                nBatteryStatus |= (canTemp.szData[3] & 0x02)? 0x40:0x00;//放电状态
                nBatteryStatus |= (canTemp.szData[7] & 0x02)? 0x80:0x00;//充电fet损坏
                m_nBatteryStatus = nBatteryStatus;
            }
        }
        vCanFrameDeque.clear();
    }
}

/*************************************************
Function: CTrackRobot::CANSendThreadFunc
Description:　USBCAN设备数据发送线程功能函数
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::CANSendThreadFunc()
{
    ROS_INFO("[CANSendThreadFunc] start");
    CanFrame canFrame;
    std::deque<CanFrame> CanSendDeque;
    unsigned long ulSize = 0;

    while (ros::ok())
    {
        try
        {
            std::unique_lock<std::mutex> LockSendPack(m_CanTXMutex);
            if (m_dCanTXDeque.empty())
            {
                m_CanTXCondition.wait(LockSendPack);
            }

            CanSendDeque.assign(m_dCanTXDeque.begin(), m_dCanTXDeque.end());
            m_dCanTXDeque.clear();
            LockSendPack.unlock();

            ulSize = CanSendDeque.size();
            if(ulSize > 100)
            {
                ROS_WARN("[CANSendThreadFunc] send buf size is: %ld.",ulSize);
            }
            for (int i = 0; i < ulSize; i++)
            {
                canFrame = CanSendDeque[i];

                if(m_nPrintCanRTX)
                    UsbCan.PrintCanFrame(&canFrame, __FUNCTION__);
                UsbCan.SendCan(&canFrame, 1);

                this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        catch(...)
        {
            UsbCan.Reset();
            ROS_ERROR("[CANSendThreadFunc] catch can send func error");
        }

    }
}

/*************************************************
Function: CTrackRobot::CheckTaskStatus
Description: 检测任务是否完成
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::CheckTaskStatus()
{
    if(m_nControlMode != IDLE)
    {
        if(m_dDistance >= m_dTargetDis - AUTO_RUN_LOCATION_TRIM)
        {
            if(m_nMode == PV || m_bSingleMove)
            {
                ROS_INFO("[CheckTaskStatus] run Quick Stop");
                QuickStop();
                this_thread::sleep_for(std::chrono::milliseconds(200));
            }

            if(WaitForMotionStop(5))
            {
                PublishStatus(TASK_DONE);
                m_bPubPosData = false;
            }
            else
                PublishStatus(ERROR);

            m_bSingleMove = false;
            m_nControlMode = IDLE;
            ROS_DEBUG("[CheckTaskStatus] forward or backward m_dDistance=%lf",m_dDistance);
        }
    }
}

/*************************************************
Function: CTrackRobot::UpdateMotorInfoTimerFunc
Description:　状态监测TIMER
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::UpdateMotorInfoTimerFunc()
{
    if(1 == m_nUseImuOdom)
    {
        PublishOdom();
    }
    if(!m_bIsMotorInit || m_bIsEStopButtonPressed)
        return;

    SendGetMotorInfoCmd();
}

/*************************************************
Function: CTrackRobot::UpdateMotorInfoTimerFunc
Description:　状态监测TIMER
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::MonitorStatusThreadFunc()
{
    ROS_INFO("[MonitorStatusThreadFunc] start");

    while (ros::ok())
    {
        if(m_nControlMode == JOY)
        {
            CheckJoyStatus();
        }
        else
        {
            CheckTaskStatus();
        }

        CheckCanStatus();

        CheckEStopStatus();

        if(!m_bIsMotorInit || m_bIsEStopButtonPressed)
        {}
        else
        {
            //非位置信息的速度处理:匀加速 or 匀减速
            if(m_nMode == PV || m_bSingleMove)
                AccelerationOrDeceleration();
        }

        this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    ROS_INFO("[MonitorStatusThreadFunc] end");
}

/*************************************************
Function: CTrackRobot::AccelerationOrDeceleration
Description: 匀加速 and 匀减速
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::AccelerationOrDeceleration()
{
    if(m_nControlMode == TASK || m_nControlMode == ALARM)
    {
        std::unique_lock<std::mutex> Lock(m_CurrentSpeedMutex);

        double dEndSpeed = 0.05;
        if( !m_bIsMotorPause && (abs(m_dTargetSpeed) > dEndSpeed) && (m_dDistance >= (m_dTargetDis - m_dEndDis)) && m_nControlMode != JOY)
        {
            if(m_nDirection == FORWARD)
            {
                m_dTargetSpeed = m_dTargetSpeed > dEndSpeed ? dEndSpeed : m_dTargetSpeed;
                ROS_DEBUG("[AccelerationOrDeceleration] deceleration set speed:%f",m_dTargetSpeed);
            }
            else if(m_nDirection == BACKWARD)
            {
                m_dTargetSpeed = abs(m_dTargetSpeed) > dEndSpeed ? -dEndSpeed : m_dTargetSpeed;
                ROS_DEBUG("[AccelerationOrDeceleration] deceleration set speed:%f",m_dTargetSpeed);
            }
        }
        Lock.unlock();
    }

    SetMotorSpeed(m_dTargetSpeed);
}

/*************************************************
Function: CTrackRobot::CalcRobotMileage
Description: 换算机器人运动里程
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::CalcRobotMileage()
{
    int nMotor_One_Displacement=0, nMotor_Two_Displacement=0;

    //32位数值溢出
    if(m_nDirection == FORWARD)
    {
        if(m_nMode == PV || m_bSingleMove)
        {
            if(m_CurrentMotorPos.nMotorOnePos < m_DynamicOrigin.nMotorOnePos)
            {
                nMotor_One_Displacement = 0x7FFFFFFF - m_DynamicOrigin.nMotorOnePos;
                nMotor_One_Displacement += m_CurrentMotorPos.nMotorOnePos - 0x80000000 + 1;
            }
            else
            {
                nMotor_One_Displacement = m_CurrentMotorPos.nMotorOnePos-m_DynamicOrigin.nMotorOnePos;
            }
        }

        if(m_CurrentMotorPos.nMotorTwoPos < m_DynamicOrigin.nMotorTwoPos)
        {
            nMotor_Two_Displacement = 0x7FFFFFFF - m_DynamicOrigin.nMotorTwoPos;
            nMotor_Two_Displacement += m_CurrentMotorPos.nMotorTwoPos - 0x80000000 + 1;
        }
        else
        {
            nMotor_Two_Displacement = m_CurrentMotorPos.nMotorTwoPos-m_DynamicOrigin.nMotorTwoPos;
        }
    }
    else if(m_nDirection == BACKWARD)
    {
        if(m_nMode == PV || m_bSingleMove)
        {
            if(m_CurrentMotorPos.nMotorOnePos > m_DynamicOrigin.nMotorOnePos)
            {
                nMotor_One_Displacement = m_DynamicOrigin.nMotorOnePos - 0x80000000;
                nMotor_One_Displacement += 0x7FFFFFFF - m_CurrentMotorPos.nMotorOnePos + 1;
            }
            else
            {
                nMotor_One_Displacement = m_DynamicOrigin.nMotorOnePos-m_CurrentMotorPos.nMotorOnePos;
            }
        }

        if(m_CurrentMotorPos.nMotorTwoPos > m_DynamicOrigin.nMotorTwoPos)
        {
            nMotor_Two_Displacement = m_DynamicOrigin.nMotorTwoPos - 0x80000000;
            nMotor_Two_Displacement += 0x7FFFFFFF - m_CurrentMotorPos.nMotorTwoPos + 1;
        }
        else
        {
            nMotor_Two_Displacement = m_DynamicOrigin.nMotorTwoPos-m_CurrentMotorPos.nMotorTwoPos;
        }
    }

    if(m_nMode == PV || m_bSingleMove)
    {
        double dDistance = (nMotor_One_Displacement/(PULSES_OF_MOTOR_ONE_TURN*MOTOR_TRANSMISSION))*M_PI*actual_tire_diameter/1000;
        m_dDistance = ((nMotor_Two_Displacement/(PULSES_OF_MOTOR_ONE_TURN*MOTOR_TRANSMISSION))*M_PI*actual_tire_diameter/1000 + dDistance) / 2;
    }
    else if(m_nMode == PP)
    {
        m_dDistance = (nMotor_Two_Displacement/(PULSES_OF_MOTOR_ONE_TURN*MOTOR_TRANSMISSION))*M_PI*actual_tire_diameter/1000;
    }
}

/*************************************************
Function: CTrackRobot::MotorServoInit
Description:　电机伺服初始化函数
Input: void
Output: void
Others: void
**************************************************/
int CTrackRobot::MotorServoInit()
{
    ROS_DEBUG("[MotorServoInit] start.");
    m_bIsMotorInit = false;

    if(GpioControl(R_LIGHT_OFF) == -1)
    {
        ROS_ERROR("[MotorServoInit] turn off red light failed");
        return -1;
    }
    if(GpioControl(G_LIGHT_OFF) == -1)
    {
        ROS_ERROR("[MotorServoInit] turn off green light failed");
        return -1;
    }
    if(GpioControl(B_LIGHT_OFF) == -1)
    {
        ROS_ERROR("[MotorServoInit] turn off blue light failed");
        return -1;
    }
    if(GpioControl(R_LIGHT_ON) == -1)
    {
        ROS_ERROR("[MotorServoInit] turn on red light failed");
        return -1;
    }

    if(m_bIsEStopButtonPressed)
    {
        ROS_WARN("[MotorServoInit] motor init failed, the E-Stop button is pressed");
        return -1;
    }

    m_bIsInitializing = true;
    if(RestartMotorServo() == -1)
    {
        ROS_ERROR("[MotorServoInit] start motor servo failed");
        m_bIsInitializing = false;
        return -1;
    }

    this_thread::sleep_for(std::chrono::milliseconds(1000));

    m_DynamicOrigin.nMotorOnePos = 0;
    m_DynamicOrigin.nMotorTwoPos = 0;

    m_tTimeOfLastJoyCmd.tv_sec = 0;
    m_tTimeOfLastJoyCmd.tv_nsec = 0;

    m_tMotorOneLastRespTime.tv_sec = 0;
    m_tMotorOneLastRespTime.tv_nsec = 0;
    m_tMotorTwoLastRespTime.tv_sec = 0;
    m_tMotorTwoLastRespTime.tv_nsec = 0;

    m_MotorOne.SetMotorId(1);
    m_MotorTwo.SetMotorId(2);

    m_MotorOne.SetMotorControlMode(PV);
    m_MotorTwo.SetMotorControlMode(m_nMode);

    int nCanDataLen = 40;
    int nCount = 0;
    bool bIsCheck = true;

    CanFrame initCanData[nCanDataLen];
    CanFrame initCheckCanData[nCanDataLen];

    memset(initCanData, 0, sizeof(CanFrame)*nCanDataLen);
    memset(initCheckCanData, 0, sizeof(CanFrame)*nCanDataLen);

    SendCanFrame(0, 1, 0x01);//0x01 启动节点

    m_MotorOne.InitMotorCmdPack(initCanData);
    nCount = m_MotorOne.InitMotorCheckDataPack(initCheckCanData);

    if(SendCanFrame(initCanData, nCount, initCheckCanData, bIsCheck) == -1)
    {
        ROS_ERROR("[MotorServoInit] motor one init error");
        m_bIsInitializing = false;
        return -1;
    }

    memset(initCanData, 0, sizeof(CanFrame)*nCanDataLen);
    memset(initCheckCanData, 0, sizeof(CanFrame)*nCanDataLen);

    m_MotorTwo.InitMotorCmdPack(initCanData);
    nCount = m_MotorTwo.InitMotorCheckDataPack(initCheckCanData);

    if(SendCanFrame(initCanData, nCount, initCheckCanData, bIsCheck) == -1)
    {
        ROS_ERROR("[MotorServoInit] motor two init error");
        m_bIsInitializing = false;
        return -1;
    }

    if(SetRobotAcceleration(m_dPvAcceleration) == -1)
    {
        ROS_ERROR("[MotorServoInit] init set Acceleration error");
        m_bIsInitializing = false;
        return -1;
    }
    if(SetRobotDeceleration(m_dPvAcceleration) == -1)
    {
        ROS_ERROR("[MotorServoInit] init set Deceleration error");
        m_bIsInitializing = false;
        return -1;
    }

    if(GpioControl(R_LIGHT_OFF) == -1)
    {
        ROS_ERROR("[MotorServoInit] turn off red light failed");
        m_bIsInitializing = false;
        return -1;
    }

    if(GpioControl(G_LIGHT_ON) == -1)
    {
        ROS_ERROR("[MotorServoInit] turn on green light failed");
        m_bIsInitializing = false;
        return -1;
    }

    m_bIsMotorInit = true;
    m_bIsInitializing = false;

    PublishStatus(TASK_DONE);
    ROS_INFO("[MotorServoInit] motor init succeed");

    this_thread::sleep_for(std::chrono::milliseconds(100));
    UpdateOriginPosition();
    return 1;
}

/*************************************************
Function: CTrackRobot::SetMotorAcceleration
Description: 设置电机伺服的加速度 A
Input: float fVelocity 机器人运动减速度,单位 m/s^2
Output: void
Others: void
**************************************************/
int CTrackRobot::SetRobotAcceleration(double dAcceleration)
{
    CanFrame AccelerationCanData[2];
    CanFrame AccelerationCheckCanData[2];
    memset(AccelerationCanData, 0, sizeof(CanFrame)*2);
    memset(AccelerationCheckCanData, 0, sizeof(CanFrame)*2);

    int nCount = 0;
    m_MotorOne.SetAccelerationCmdPack(AccelerationCanData, dAcceleration);
    nCount = m_MotorOne.SetAccelerationCheckDataPack(AccelerationCheckCanData);

    if(SendCanFrame(AccelerationCanData, nCount, AccelerationCheckCanData, true)== -1)
    {
        ROS_ERROR("[SetRobotAcceleration] set motor one Acceleration error");
        return -1;
    }

    memset(AccelerationCanData, 0, sizeof(CanFrame)*2);
    memset(AccelerationCheckCanData, 0, sizeof(CanFrame)*2);

    m_MotorTwo.SetAccelerationCmdPack(AccelerationCanData, dAcceleration);
    nCount = m_MotorTwo.SetAccelerationCheckDataPack(AccelerationCheckCanData);

    if(SendCanFrame(AccelerationCanData, nCount, AccelerationCheckCanData, true)== -1)
    {
        ROS_ERROR("[SetRobotAcceleration] set motor two Acceleration error");
        return -1;
    }
    return 1;
}

/*************************************************
Function: CTrackRobot::SetMotorAcceleration
Description: 设置电机伺服的加速度 A
Input: float fVelocity 机器人运动减速度,单位 m/s^2
Output: void
Others: void
**************************************************/
int CTrackRobot::SetRobotDeceleration(double dDeceleration)
{
    CanFrame DecelerationCanData[2];
    CanFrame DecelerationCheckCanData[2];
    memset(DecelerationCanData, 0, sizeof(CanFrame)*2);
    memset(DecelerationCheckCanData, 0, sizeof(CanFrame)*2);

    int nCount = 0;
    m_MotorOne.SetDecelerationCmdPack(DecelerationCanData, dDeceleration);
    nCount = m_MotorOne.SetDecelerationCheckDataPack(DecelerationCheckCanData);

    if(SendCanFrame(DecelerationCanData, nCount, DecelerationCheckCanData, true)== -1)
    {
        ROS_ERROR("[SetRobotDeceleration] set motor one Deceleration error");
        return -1;
    }

    memset(DecelerationCanData, 0, sizeof(CanFrame)*2);
    memset(DecelerationCheckCanData, 0, sizeof(CanFrame)*2);

    m_MotorTwo.SetDecelerationCmdPack(DecelerationCanData, dDeceleration);
    nCount = m_MotorTwo.SetDecelerationCheckDataPack(DecelerationCheckCanData);

    if(SendCanFrame(DecelerationCanData, nCount, DecelerationCheckCanData, true)== -1)
    {
        ROS_ERROR("[SetRobotDeceleration] set motor two Deceleration error");
        return -1;
    }
    return 1;
}

/*************************************************
Function: CTrackRobot::ForwardMotion
Description: 机器人向正方向运动
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::ForwardMotion()
{
    m_nDirection = FORWARD;
    actual_tire_diameter = forward_diameter;

    if(m_nMode == PP && !m_bSingleMove)
    {
        SetNewPos(m_dTargetDis);
    }
    else
    {
        m_bCheckAlarmData = false;
        m_dTargetSpeed = m_dTargetSpeedTemp;

        //m_dEndDis需要判断 处理目标距离无法完成抵达目标速度的正常匀加速和匀减速
        m_dEndDis = (m_dTargetSpeed*m_dTargetSpeed)/(2*m_dPvAcceleration) + END_DIS_AMEND;

        double dTargetDis = m_dTargetDis;

        if(dTargetDis <= 2*m_dEndDis - END_DIS_AMEND)
        {
            m_dEndDis = dTargetDis*END_DIS_MODIFICATION;
        }

        ROS_DEBUG("[ForwardMotion] m_dEndDis :%f",m_dEndDis);
    }
}

/*************************************************
Function: CTrackRobot::BackwardMotion
Description: 机器人向正方向反向运动
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::BackwardMotion()
{
    m_nDirection = BACKWARD;

    actual_tire_diameter = back_diameter;

    if(m_nMode == PP && !m_bSingleMove)
    {
        SetNewPos(-m_dTargetDis);
    }
    else
    {
        m_bCheckAlarmData = false;
        m_dTargetSpeed = m_dTargetSpeedTemp;

        //m_dEndDis需要判断 处理目标距离无法完成抵达目标速度的正常匀加速和匀减速
        m_dEndDis = (m_dTargetSpeed*m_dTargetSpeed)/(2*m_dPvAcceleration) + END_DIS_AMEND;

        double dTargetDis = m_dTargetDis;

        if(dTargetDis <= 2*m_dEndDis - END_DIS_AMEND)
        {
            m_dEndDis = dTargetDis*END_DIS_MODIFICATION;
        }

        ROS_DEBUG("[BackwardMotion] m_dEndDis :%f",m_dEndDis);
    }
}

/*************************************************
Function: CTrackRobot::MotorSpeedUp
Description: 电机运动加速
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::MotorSpeedUp()
{
    std::unique_lock<std::mutex> Lock(m_CurrentSpeedMutex);
    if(m_nMode == PP)
    {
        m_dTargetSpeed += SPEED_INCREMENT;
    }
    else if(m_nMode == PV)
    {
        if(m_nDirection == FORWARD)
        {
            m_dTargetSpeed += SPEED_INCREMENT;
        }
        else
        {
            m_dTargetSpeed -= SPEED_INCREMENT;
        }
    }
    Lock.unlock();
}

/*************************************************
Function: CTrackRobot::MotorSpeedDown
Description: 电机运动减速
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::MotorSpeedDown()
{
    std::unique_lock<std::mutex> Lock(m_CurrentSpeedMutex);
    if(m_nMode == PP)
    {
        m_dTargetSpeed -= SPEED_INCREMENT;
        if(m_dTargetSpeed <= 0)
            m_dTargetSpeed = 0;

    }
    else if(m_nMode == PV)
    {
        if(m_nDirection == FORWARD)
        {
            m_dTargetSpeed -= SPEED_INCREMENT;
            if(m_dTargetSpeed <= 0)
            {
                m_dTargetSpeed = 0;
            }
        }
        else
        {
            m_dTargetSpeed -= SPEED_INCREMENT;
            if(m_dTargetSpeed >= 0)
            {
                m_dTargetSpeed = 0;
            }
        }
    }
    Lock.unlock();
}

/*************************************************
Function: CTrackRobot::SetMotorSpeed
Description: 设置电机运动速度
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::SetMotorSpeed(double dSpeed)
{
    CanFrame SpeedCanData[2];
    CanFrame *pCanData = SpeedCanData;
    int nCount = 0;

    if(m_nMode == PV || m_bSingleMove)
    {
        nCount += m_MotorOne.SetSpeedPdoCmdPack(pCanData, dSpeed);
        pCanData = SpeedCanData + nCount;
    }

    if(!m_bSingleMove)
        nCount += m_MotorTwo.SetSpeedPdoCmdPack(pCanData, dSpeed);

    SendCanFrame(SpeedCanData, nCount);
}

/*************************************************
Function: CTrackRobot::SendCmdOfGetMotorTemp
Description: 发送获取电机伺服温度报文
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::SendCmdOfGetMotorTemp()
{
    CanFrame CanData[2];
    CanFrame *pCanData = CanData;
    int nCount = 0;
    nCount += m_MotorOne.GetMotorTempPack(pCanData);
    pCanData = CanData + nCount;
    nCount += m_MotorTwo.GetMotorTempPack(pCanData);

    SendCanFrame(CanData, nCount);
}

/*************************************************
Function: CTrackRobot::SendCmdOfGetMotorErrorCode
Description: 发送获取电机伺服温度报文
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::SendCmdOfGetMotorErrorCode()
{
    CanFrame CanData[2];
    CanFrame *pCanData = CanData;
    int nCount = 0;
    nCount += m_MotorOne.GetErrorCodeCmdPack(pCanData);
    pCanData = CanData + nCount;
    nCount += m_MotorTwo.GetErrorCodeCmdPack(pCanData);

    SendCanFrame(CanData, nCount);
}

/*************************************************
Function: CTrackRobot::SendHeartBeatCanFrame
Description: 设置电机运动速度
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::SendHeartBeatCanFrame()
{
    SendCanFrame(MASTER_HEART_BEAT_ID, 0, 0);
}

/*************************************************
Function: CTrackRobot::SetMotorSpeed
Description: 设置电机运动速度
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::SendCmdOfGetBatteryInfo()
{
    SendCanFrame(0x16, 8, 0x16BB00000000007E);
    SendCanFrame(0x16, 8, 0x16BE00000000007E);
}

/*************************************************
Function: CTrackRobot::SetNewPos
Description: 位置模式下,设置电机的新的位置点
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::SetNewPos(double dDistance)
{
    CanFrame PosCanData[5];
    CanFrame *pCanData = PosCanData;

    int nType = RELATIVE;

    int nCount = m_MotorTwo.SetNewPosCmdPack(pCanData, nType, dDistance);

    pCanData = PosCanData + nCount;
    m_MotorOne.SetSdoCanFrame(pCanData, SdoControlWords[ENABLE_OPERATION]);
    nCount ++;

    SendCanFrame(PosCanData, nCount);
}

/*************************************************
Function: CTrackRobot::QuickStop
Description: 电机运动停止，急停
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::QuickStop()
{
    m_dTargetSpeed = 0;
    SetMotorSpeed(m_dTargetSpeed);

    CanFrame StopCanData[2];
    CanFrame *pCanData = StopCanData;
    int nCount = 0;
    nCount += m_MotorOne.QuickStopCmdPack(pCanData);
    pCanData = StopCanData + nCount;
    nCount += m_MotorTwo.QuickStopCmdPack(pCanData);

    SendCanFrame(StopCanData, nCount);
//    ROS_DEBUG("[QuickStop] send quick stop cmd");
}

/*************************************************
Function: CTrackRobot::StopMotion
Description: 电机运动停止，按照设置的匀减速，速度减至0
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::StopMotion(bool bJoy)
{
    if(m_nMode == PV)
    {
        m_dTargetSpeed = 0;
    }
    else if(m_nMode == PP)
    {
        CanFrame StopCanData[2];
        CanFrame *pCanData = StopCanData;
        int nCount = 0;
        nCount += m_MotorOne.StopCmdPack(pCanData);
        pCanData = StopCanData + nCount;
        nCount += m_MotorTwo.StopCmdPack(pCanData);

        SendCanFrame(StopCanData, nCount);
    }
    if(!bJoy)
        m_bIsMotorPause = true;
}

/*************************************************
Function: CTrackRobot::MotorPowerOff
Description: 电机伺服控制掉电
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::MotorPowerOff()
{
    SendCanFrame(0x601, 8, 0x2B40600006000000);//断电
    SendCanFrame(0x602, 8, 0x2B40600006000000);
    m_bMotorRunning = false;
}

/*************************************************
Function: CTrackRobot::ClearWarn
Description: 清除伺服警告
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::ClearWarn()
{
    if(m_bIsMotorInit)
    {
        ROS_INFO("[ClearWarn]m_bIsMotorInit = %d, return.",m_bIsMotorInit);
        return;
    }
    int nSize = 2;
    CanFrame ClearCanData[nSize];
    CanFrame ClearCanCheckData[nSize];
    bool bIsCheck = true;

    memset(ClearCanData, 0, sizeof(CanFrame)*nSize);
    memset(ClearCanCheckData, 0, sizeof(CanFrame)*nSize);

    m_MotorOne.ClearWarnPack(ClearCanData);
    int nCount = m_MotorOne.ClearWarnCheckCmdPack(ClearCanCheckData);

    if(SendCanFrame(ClearCanData, nCount, ClearCanCheckData, bIsCheck) == -1)
    {
        ROS_ERROR("[ClearWarn] motor one clear warn error");
    }

    memset(ClearCanData, 0, sizeof(CanFrame)*nSize);
    memset(ClearCanCheckData, 0, sizeof(CanFrame)*nSize);


    m_MotorTwo.ClearWarnPack(ClearCanData);
    nCount = m_MotorTwo.ClearWarnCheckCmdPack(ClearCanCheckData);

    if(SendCanFrame(ClearCanData, nCount, ClearCanCheckData, bIsCheck) == -1)
    {
        ROS_ERROR("[ClearWarn] motor two clear warn error");
    }

    m_sMotorOneStatus = "normal";
    m_sMotorTwoStatus = "normal";
}

/*************************************************
Function: CTrackRobot::SendGetMotorPosCmd
Description: 获取电机的当前位置值,绝对位移
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::SendGetMotorInfoCmd()
{
    SendCanFrame(GET_MOTOR_INFO_PDO_ID, 0, 0);
}

/*************************************************
Function: CTrackRobot::SendCanFrame
Description: can数据发送函数
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::SendCanFrame(unsigned ID, unsigned char DataLen, ULL frameData)
{
    CanFrame canFrame;

    UsbCan.SetCanFrame(&canFrame, ID, DataLen, frameData);

    std::unique_lock<std::mutex> LockSendCanFrame(m_CanTXMutex);
    m_dCanTXDeque.push_back(canFrame);
    m_CanTXCondition.notify_one();
}

int CTrackRobot::SendCanFrame(CanFrame *sendCanData, int nCount, CanFrame *checkCanData, bool bIsCheck)
{
    if(bIsCheck)
    {
        std::unique_lock<std::mutex> Lock(m_SdoCheckMutex);
        m_SdoCheckBuf = new CanFrame[nCount];
        memcpy(m_SdoCheckBuf, checkCanData, sizeof(CanFrame)*nCount);
        Lock.unlock();

        m_nSdoCheckCount = 0;
        m_bCheckSdo = true;
    }

    std::unique_lock<std::mutex> LockSendCanFrame(m_CanTXMutex);
    for(int i=0; i<nCount; i++)
    {
        m_dCanTXDeque.push_back(sendCanData[i]);
    }
    m_CanTXCondition.notify_one();
    LockSendCanFrame.unlock();

    if(bIsCheck)
    {
        timespec StartTime;
        timespec_get(&StartTime, TIME_UTC);

        while(m_nSdoCheckCount < nCount)
        {
            this_thread::sleep_for(std::chrono::milliseconds(10));

            timespec CurrentTime;
            timespec_get(&CurrentTime, TIME_UTC);

            long pollInterval=((CurrentTime.tv_sec - StartTime.tv_sec) * 1000000000 \
                                 + (CurrentTime.tv_nsec - StartTime.tv_nsec)) / 1000000;
            if(pollInterval > (nCount * 20) || !m_bCheckSdo)
            {
                if(m_bCheckSdo)
                    ROS_WARN("[SendCanFrame] receive data time out");
                m_nSdoCheckCount = 0;
                m_bCheckSdo = false;
                return -1;
            }
        }

        std::unique_lock<std::mutex> Lock1(m_SdoCheckMutex);
        delete m_SdoCheckBuf;
        m_SdoCheckBuf = nullptr;
        Lock1.unlock();

        m_nSdoCheckCount = 0;
        m_bCheckSdo = false;
    }
    return 1;
}

/*************************************************
Function: CTrackRobot::PublishOdom
Description: can帧数据打印函数
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::PublishOdom()
{
    nav_msgs::Odometry OdomMsg;

    OdomMsg.header.stamp = ros::Time::now();
    OdomMsg.header.frame_id = "odom";

    if(m_bIsEStopButtonPressed)
    {
        m_dCurrentSpeed = 0;
    }

    OdomMsg.twist.twist.linear.x = m_dCurrentSpeed;

    OdomMsg.twist.covariance[0] = 1e-8;
    OdomMsg.twist.covariance[7] = 1e-8;
    OdomMsg.twist.covariance[14] = 1e-8;
    OdomMsg.twist.covariance[21] = 1e-8;
    OdomMsg.twist.covariance[28] = 1e-8;
    OdomMsg.twist.covariance[35] = 1e-8;

    m_OdomPublisher.publish(OdomMsg);
}

/*************************************************
Function: CTrackRobot::PublishStatus
Description: can帧数据打印函数
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::PublishStatus(int nStatus)
{
    custom_msgs::GeneralTopic status;

    status.sender = "track_robot";
    status.data = vsRobotStatus[nStatus];
    status.header.stamp = ros::Time::now();
    status.header.frame_id = "status";

    m_StatusPublisher.publish(status);
}

/*************************************************
Function: CTrackRobot::PublishStatus
Description: can帧数据打印函数
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::PublishStatus(string &sStatus)
{
    custom_msgs::GeneralTopic status;

    status.sender = "track_robot";
    status.data = sStatus;
    status.header.stamp = ros::Time::now();
    status.header.frame_id = "status";

    m_StatusPublisher.publish(status);
}

/*************************************************
Function: CTrackRobot::CommandCallBack
Description: 后台控制回调函数
Input: const custom_msgs::TrainRobotControl::ConstPtr &Command, 后台控制消息
Output: void
Others: void
**************************************************/
void CTrackRobot::CommandCallBack(const custom_msgs::TrainRobotControl::ConstPtr &Command)
{
    custom_msgs::TrainRobotControlAck Ack;
    Ack.trans_id = Command->trans_id;
    Ack.header.stamp = ros::Time::now();
    Ack.header.frame_id = Command->cmd;
    Ack.sender = "track_robot";
    Ack.ack = "succeed";

    if(!m_bIsMotorInit)
    {
        ROS_WARN("[CommandCallBack] motor is not init.");
        Ack.ack = "failed";
        Ack.data = "motor is not init";
    }
    else if(m_bIsEStopButtonPressed)
    {
        ROS_WARN("[CommandCallBack] E-Stop button is pressed.");
        Ack.ack = "failed";
        Ack.data = "E-Stop button is pressed";
    }
    else if(Command->cmd == "joy_run")
    {
        if(m_bMotorRunning && m_nControlMode != JOY)
        {
            ROS_WARN("[CommandCallBack] busy, please wait for a moment");
        }
        else
        {
            m_nControlMode = JOY;

            std::unique_lock<std::mutex> lock(m_JoyTimeOutCheckMutex);
            if(m_tTimeOfLastJoyCmd.tv_sec == 0 && m_tTimeOfLastJoyCmd.tv_nsec == 0)
            {
                m_dTargetSpeed = Command->speed;
                if(m_nMode == PP)
                {
                    SetMotorSpeed(abs(m_dTargetSpeed));
                    if(Command->speed > 0)
                        SetNewPos(2000.0);
                    else
                        SetNewPos(-2000.0);
                }
                m_nDirection = m_dTargetSpeed > 0 ? FORWARD : BACKWARD;
            }
            timespec_get(&m_tTimeOfLastJoyCmd, TIME_UTC);
            lock.unlock();
        }
        return;
    }
    else if(Command->cmd == "run" || Command->cmd == "alarm_run")
    {
        m_bSingleMove = false;
        m_bPubPosData = false;

        if(m_bMotorRunning)
        {
            ROS_WARN("[CommandCallBack] robot is running, return.");
            Ack.ack = "failed";
            Ack.data = "robot is running";
        }
        else if(m_nControlMode != IDLE)
        {
            ROS_WARN("[CommandCallBack] control mode is not idle.");
            Ack.ack = "failed";
            Ack.data = "busy";
        }
        else if(Command->distance < 0.001)
        {
            Ack.data = "target dis is to small";
        }
        else
        {
            if(Command->cmd == "run")
            {
                m_bPubPosData = (Command->move_type == UPDATE_ORIGIN_MOVE_PUB) || (Command->move_type == MOVE_PUB);
                this_thread::sleep_for(std::chrono::milliseconds(100));//确保先把激光数据打开
                if(UPDATE_ORIGIN_MOVE_PUB == Command->move_type)
                {
                    m_bManagerUpdateOrigin = (Command->move_type == 1);
                    m_dRealDis = Command->real_distance;
                    m_bForwardRelocation = (Command->speed > 0);
                }
                else if(SINGLE_MOVE == Command->move_type)
                {
                    m_dTargetDis /= 2;
                    m_bSingleMove = true;
                }
            }
            else
            {
                m_bPubPosData = (Command->move_type == MOVE_PUB);
                this_thread::sleep_for(std::chrono::milliseconds(100));//确保先把激光数据打开
            }

            m_dDisTemp = 0xFFFFFFFFFFFFFFFF;

            if(Command->distance != m_dTargetDis)
            {
                ROS_INFO("[CommandCallBack] set distance :%f",Command->distance);
                m_dTargetDis = Command->distance;
            }

            UpdateOriginPosition();

            ROS_INFO("[CommandCallBack] set speed :%f",Command->speed);
            m_dTargetSpeedTemp = Command->speed;
            m_bIsMotorPause = false;
            if(m_nMode == PP)
            {
                SetMotorSpeed(abs(m_dTargetSpeedTemp));
            }
            if(m_dTargetSpeedTemp > 0)
            {
                ForwardMotion();
            }
            else
            {
                BackwardMotion();
            }

            m_nControlMode = Command->cmd == "run" ? TASK : ALARM;
        }
    }
    else if(Command->cmd == "stop")
    {
        StopMotion();

        timespec CurrentTime, LastTime;
        long pollInterval;
        timespec_get(&LastTime, TIME_UTC);
        while(m_bMotorRunning)
        {
            timespec_get(&CurrentTime, TIME_UTC);
            pollInterval=(CurrentTime.tv_sec - LastTime.tv_sec) + (CurrentTime.tv_nsec - LastTime.tv_nsec)/1000000000;
            if(pollInterval > 15)
            {
                Ack.ack = "failed";
                Ack.data = "timeout";
                break;
            }
            this_thread::sleep_for(std::chrono::seconds(1));
        }
        m_nControlMode = IDLE;
    }
    else if(Command->cmd == "update_origin")
    {
        if(Command->move_type == UPDATE_ORIGIN)
        {
            m_nRobotOriginPulseOne = m_nRobotCurrentPulseOne;
            m_nRobotOriginPulseTwo = m_nRobotCurrentPulseTwo;
        }
        else if(Command->move_type == UPDATE_ORIGIN_PLUS)
        {
            actual_tire_diameter = forward_diameter;
            m_nRobotOriginPulseOne = m_nRobotCurrentPulseOne + m_MotorTwo.TransMileToPulse(Command->distance);
            m_nRobotOriginPulseOne = m_nRobotCurrentPulseTwo + m_MotorTwo.TransMileToPulse(Command->distance);
        }
        else if(Command->move_type == UPDATE_ORIGIN_SUB)
        {
            actual_tire_diameter = forward_diameter;
            m_nRobotOriginPulseOne = m_nRobotCurrentPulseOne - m_MotorTwo.TransMileToPulse(Command->distance);
            m_nRobotOriginPulseTwo = m_nRobotCurrentPulseTwo - m_MotorTwo.TransMileToPulse(Command->distance);
        }
    }
    //added by btrmg for adjust mileage 2020.01.07
    else if(Command->cmd =="adjust_mileage")
    {
        ROS_INFO("[CommandCallBack] adjust expected mileage:%d and actual mileage:%d",Command->expected_mileage,Command->actual_mileage);
        if(0 != Command->actual_mileage && 0 != Command->expected_mileage)
        {
            if(Command->expected_mileage < 1000)
            {
                Ack.data = "refused, the distance is less than 1000mm";
                ROS_WARN("[CommandCallBack] adjust_mileage refused, the distance:%dmm is less than 1000mm", Command->expected_mileage);
            }
            else if(Command->speed > 0)
            {
                double dForwardDiameter = forward_diameter/Command->expected_mileage*Command->actual_mileage;
                if(abs(dForwardDiameter - forward_diameter) > m_dAdjustLimit)
                {
                    ROS_WARN("[CommandCallBack] adjust_mileage over limit, forward_diameter:%f, dForwardDiameter:%f",forward_diameter,dForwardDiameter);
                    Ack.data = "refused, over limit";
                }
                else
                {
                    forward_diameter = dForwardDiameter;
                    string str_diameter;
                    str_diameter = std::to_string(forward_diameter);
                    updata_XML("forward_diameter",str_diameter);
                    ROS_DEBUG("[CommandCallBack] adjusted forward diameter is:%f",forward_diameter);
                }
            }
            else if(Command->speed < 0)
            {
                double dBackDiameter = back_diameter/Command->expected_mileage*Command->actual_mileage;
                if(abs(dBackDiameter - back_diameter) > m_dAdjustLimit)
                {
                    ROS_WARN("[CommandCallBack] adjust_mileage over limit, back_diameter:%f, dBackDiameter:%f",back_diameter,dBackDiameter);
                    Ack.data = "refused, over limit";
                }
                else
                {
                    back_diameter = dBackDiameter;
                    string str_diameter;
                    str_diameter = std::to_string(back_diameter);
                    updata_XML("back_diameter",str_diameter);
                    ROS_DEBUG("[CommandCallBack] adjusted back diameter is :%f",back_diameter);
                }
            }
        }
    }
    //added end
    else
    {
        Ack.ack = "failed";
        Ack.data = "unknown command";
        ROS_INFO("[CommandCallBack] unknown command: %s",Command->cmd.c_str());
    }
    m_MoveAckPublisher.publish(Ack);
    if((Command->cmd == "run" || Command->cmd == "alarm_run") && Command->distance < 0.001)
    {
        PublishStatus(TASK_DONE);
    }
    ROS_DEBUG("[CommandCallBack] cdm:%s, ack:%s, data:%s", Command->cmd.c_str(), Ack.ack.c_str(), Ack.data.c_str());
}

/*************************************************
Function: CTrackRobot::OdomCallBack
Description: 滤波后的历程消息
Input: nav_msgs::Odometry::ConstPtr &OdomMsg , 滤波后的里程消息
Output: void
Others: void
**************************************************/
void CTrackRobot::OdomCallBack(const nav_msgs::Odometry::ConstPtr &OdomMsg)
{
    m_dCurrentOdom = OdomMsg->pose.pose.position.x;
    m_dDistance = m_dCurrentOdom - m_dOdom;

    if(m_nDirection == BACKWARD)
    {
        m_dDistance =  m_dOdom - m_dCurrentOdom;
    }
    if(m_bIsUpdateOrigin)
    {
        m_dOdom = m_dCurrentOdom;
        m_bIsUpdateOrigin = false;
    }
}

/*************************************************
Function: CTrackRobot::BatteryServiceFunc
Description: 电池状态查询服务端
Input: custom_msgs::BatteryStatus::Request &Req 请求
        custom_msgs::BatteryStatus::Response &Resp 响应
Output: true or false
Others: void
**************************************************/
bool CTrackRobot::BatteryServiceFunc(custom_msgs::BatteryStatus::Request &Req, custom_msgs::BatteryStatus::Response &Resp)
{
    Resp.response = Req.request + "_response";

    SendCanFrame(0x16, 8, 0x16BB00000000007E);

    timespec StartTime;
    timespec_get(&StartTime, TIME_UTC);
    this_thread::sleep_for(std::chrono::milliseconds(20));

    while(ros::ok())
    {
        //此处加锁是规避多个client高并发请求时引入的数据混乱
        std::unique_lock<std::mutex> lock(m_ButteryStatusMutex);
        int nButteryPower = m_nBatteryPower;
        lock.unlock();
        if(nButteryPower)
        {
            Resp.data = "{";
            Resp.data += "\"battery_power\":" + to_string(nButteryPower);
            Resp.data += "}";

            std::unique_lock<std::mutex> locker(m_ButteryStatusMutex);
            m_nBatteryPower = 0;
            locker.unlock();

            return true;
        }
        else
        {
            timespec CurrentTime;
            timespec_get(&CurrentTime, TIME_UTC);

            long pollInterval=((CurrentTime.tv_sec - StartTime.tv_sec) * 1000000000 \
                                 + (CurrentTime.tv_nsec - StartTime.tv_nsec)) / 1000000;
            if(pollInterval > 1000)
            {
                Resp.data = "timeout error!";
                return true;
            }
            SendCanFrame(0x16, 8, 0x16BB00000000007E);
            this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
}

/*************************************************
Function: CTrackRobot::CurrentPosServiceFunc
Description: 获取当前位置信息
Input: custom_msgs::CurrentPosition::Request &Req 请求
        custom_msgs::CurrentPosition::Response &Resp 响应
Output: true or false
Others: void
**************************************************/
bool CTrackRobot::CurrentPosServiceFunc(custom_msgs::CurrentPosition::Request &Req, custom_msgs::CurrentPosition::Response &Resp)
{
    Resp.response = Req.request + "_response";

    std::unique_lock<std::mutex> locker(m_CurrentPosScanBufMutex);
    if(!m_bGetAxesData && !m_dCurrentPosScanBuf.empty())
    {
        m_dCurrentPosScanBuf.clear();
//        ROS_DEBUG("[CurrentPosServiceFunc]m_dCurrentPosScanBuf.clear()");
    }
    locker.unlock();

    m_bGetAxesData = true;

    std::deque<sScanBuf> dScanBufTemp;

    timespec StartTime;
    timespec_get(&StartTime, TIME_UTC);
    this_thread::sleep_for(std::chrono::milliseconds(20));

    while(ros::ok())
    {
        dScanBufTemp.clear();
        std::unique_lock<std::mutex> lock(m_CurrentPosScanBufMutex);
        dScanBufTemp.assign(m_dCurrentPosScanBuf.begin(), m_dCurrentPosScanBuf.end());
        lock.unlock();

        if(dScanBufTemp.size() >= CURRENT_DATA_TOTAL_NUM)
        {
            m_bGetAxesData = false;
            break;
        }
        else
        {
            timespec CurrentTime;
            timespec_get(&CurrentTime, TIME_UTC);

            long pollInterval=((CurrentTime.tv_sec - StartTime.tv_sec) * 1000000000 \
                                 + (CurrentTime.tv_nsec - StartTime.tv_nsec)) / 1000000;
            if(pollInterval > 1000)
            {
                ROS_ERROR("[CurrentPosServiceFunc] timeout error");
                Resp.data = "timeout error!";
                m_bGetAxesData = false;
                return true;
            }

            this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    //二维数组数据
    float fScanMatrix[CURRENT_DATA_TOTAL_NUM][LASER_MAX_DATA_LEN]={0};
    sScanBuf SumRanges;
    memset(&SumRanges, 0 , sizeof(sScanBuf));
    //将激光数据copy至二维数组当中
    for(int row=0; row < CURRENT_DATA_TOTAL_NUM; row++)
    {
        sScanBuf sBufTemp = dScanBufTemp[row];

        for(int column=0; column < LASER_MAX_DATA_LEN; column++)
        {
            fScanMatrix[row][column] = sBufTemp.fRanges[column];
        }
        SumRanges.x = sBufTemp.x;
    }
    //剔除同一角度位置的有0出现的数据,将没有0出现的数据累计求和
    for(int column=0; column<LASER_MAX_DATA_LEN; column++)
    {
        for(int row=0; row<CURRENT_DATA_TOTAL_NUM; row++)
        {
            if(fScanMatrix[row][column] == 0)
            {
                SumRanges.fRanges[column] = 0;
                break;
            }
            else
            {
                SumRanges.fRanges[column] += fScanMatrix[row][column];
            }
        }
    }
    //求均值
    for(int i=0; i<LASER_MAX_DATA_LEN; i++)
    {
        SumRanges.fRanges[i] = SumRanges.fRanges[i]/CURRENT_DATA_TOTAL_NUM;
    }

    SumRanges.ullRangeSize = LASER_MAX_DATA_LEN;

    sPositionData desPositionData = {0};
    int nDataLen = 0;

    nDataLen = TransPosBufData(desPositionData, SumRanges, true);
    Resp.x =  desPositionData.x;
    vector<float> filter_y;
    vector<float> filter_z;
    filter_y.resize((unsigned long)nDataLen);
    filter_z.resize((unsigned long)nDataLen);
    for(int i = 0; i < nDataLen; i++)
    {
        filter_y[i] = desPositionData.y[i];
        filter_z[i] = desPositionData.z[i];
    }

    FilterPoints(filter_y,filter_z,Resp.y,Resp.z);

   /*
    Resp.y.resize((unsigned long)nDataLen);
    Resp.z.resize((unsigned long)nDataLen);

    for(int i = 0; i < nDataLen; i++)
    {
        Resp.y[i] = desPositionData.y[i];
        Resp.z[i] = desPositionData.z[i];
    }
    */

    Resp.data = "succeed";
    return true;
}

/*************************************************
Function: CTrackRobot::ImuCallBack
Description: 激光节点消息回调函数,按照激光的回调频率对里程x做平均差值处理，
             并把处理后的x及激光原始数据缓存到队列当中
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::ImuCallBack(const sensor_msgs::Imu::ConstPtr &ImuData)
{
    double dRoll, dPitch, dYaw;
    tf::Matrix3x3(tf::Quaternion(ImuData->orientation.x,
                                 ImuData->orientation.y,
                                 ImuData->orientation.z,
                                 ImuData->orientation.w)).getRPY(dRoll, dPitch, dYaw);
//    ROS_INFO("[ImuCallBack] dRoll=%f, dPitch=%f, dYaw=%f",dRoll/M_PI*180, dPitch/M_PI*180, dYaw/M_PI*180);

    double dPitchAngle = 180*dPitch/M_PI;
    m_dImuPitchDeque.push_back(dPitchAngle);
    if(m_dImuPitchDeque.size() > m_nImuBufSize)
        m_dImuPitchDeque.pop_front();

    dPitchAngle = 0.0;
    for(auto i : m_dImuPitchDeque)
    {
        dPitchAngle += i;
    }

    m_dPitchAngle = dPitchAngle / m_dImuPitchDeque.size();

    m_dRealScanTitle = m_dScanTitle - m_dPitchAngle;
    m_dInitialAngle = 180.0-m_dRealScanTitle-(90.0-LASER_ANGLE_RANGE/2)-LASER_EXCISION_ANGLE;
}


/*************************************************
Function: CTrackRobot::ScanCallBack
Description: 激光节点消息回调函数,按照激光的回调频率对里程x做平均差值处理，
             并把处理后的x及激光原始数据缓存到队列当中
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::ScanThreadFunc()
{
    asr_sick_lms_400 lms_ = asr_sick_lms_400 ("192.168.1.100", 2111, 0);

    // Attempt to connect to the laser unit
    if (lms_.Connect () != 0)
    {
        ROS_ERROR("[ScanThreadFunc] Connecting to SICK LMS4000 on 192.168.1.100:2111 filed");
    }
    ROS_INFO ("[ScanThreadFunc] Connecting to SICK LMS4000 on 192.168.1.100:2111 succeed");

    // Stop the measurement process (in case it's running from another instance)
    lms_.StopMeasurement();


    // Login to userlevel 4
    if (lms_.SetUserLevel (4, "81BE23AA") != 0)
    {
        ROS_ERROR("[ScanThreadFunc] Unable to change user level to 'Service' using ");
    }
    else
    {
        ROS_INFO("[ScanThreadFunc]Set User Level to 'Service'");
    }

    // Start Continous measurements
    lms_.StartMeasurement (true);
    lms_.sMNRUN ();
    lms_.LMDscandata (1);
    ROS_INFO("[ScanThreadFunc]start read measurement data");

    while (ros::ok ())
    {
        sensor_msgs::LaserScan scanData = lms_.ReadMeasurement();
        //以下情况直接返回：
        // 前端没有获取当前数据 并且 ①机器人停止状态 ②机器人做返程运动并且不记录返程数据 ③机器人控制模式为手柄模式 ④
        if(!m_bGetAxesData)
        {
//            if(!m_bMotorRunning || !m_bPubPosData)
            if(!m_bPubPosData)
            {
                if(!m_dScanSrcBufDeque.empty())
                {
                    std::unique_lock<std::mutex> lock(m_ScanSrcBufMutex);
                    m_dScanSrcBufDeque.clear();
                    lock.unlock();
                }
//                this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
        }
//        sensor_msgs::LaserScan scanData = lms_.LMDscandataOneTimes();

        if (scanData.ranges.empty())
            continue;

        double dDistance, d_x;
        ULL ullRangeSize = scanData.ranges.size();

        sScanBuf ScanBuf;
        memset(&ScanBuf, 0 , sizeof(sScanBuf));
        memcpy(&(ScanBuf.fRanges[0]), &(scanData.ranges[0]), sizeof(ScanBuf.fRanges[0])*ullRangeSize);

        dDistance = m_dDistance;
        ScanBuf.x = dDistance;
        ScanBuf.ullRangeSize = ullRangeSize;
//        ROS_INFO("[ScanThreadFunc] get scan data");

        //前台获取当前位置数据的service用
        if(m_bGetAxesData)
        {
            //按键获取当前位置信息（x y z）
            std::unique_lock<std::mutex> locker(m_CurrentPosScanBufMutex);
            m_dCurrentPosScanBuf.push_back(ScanBuf);
            if(m_dCurrentPosScanBuf.size() > CURRENT_DATA_TOTAL_NUM)
            {
                m_dCurrentPosScanBuf.pop_front();
            }
            locker.unlock();

            if(!m_bMotorRunning)
                continue;
        }

        m_dScanSrcBufDeque.push_back(ScanBuf);

//涉及里程的平均插值，第一个位置数据单独处理（直接保存）
        if(m_dDisTemp == 0xFFFFFFFFFFFFFFFF)
        {
            std::unique_lock<std::mutex> lock(m_ScanManBufMutex);
            m_dScanManBufDeque.push_back(ScanBuf);
            lock.unlock();
            m_ScanBufCondition.notify_one();

            m_dScanSrcBufDeque.clear();
            m_dDisTemp = dDistance;
            continue;
        }
//对x做平均插值
        if(abs(dDistance - m_dDisTemp) > 0.0001)
        {
            ULL ullRangeCount =  m_dScanSrcBufDeque.size();
//        ROS_DEBUG("dDistance=%f,m_dDisTemp=%f,ullRangeCount=%lld",dDistance,m_dDisTemp,ullRangeCount);

            std::unique_lock<std::mutex> lock(m_ScanManBufMutex);

            for(int i=0; i<ullRangeCount; i++)
            {
                sScanBuf ScanBufTemp = m_dScanSrcBufDeque[i];
                if(ullRangeCount == 1)
                {
                    d_x = dDistance;
                }
                else
                {
                    d_x = m_dDisTemp + ((dDistance-m_dDisTemp)/ullRangeCount)*(i+1);
                }

                if(abs(d_x - m_dDisTemp) < 0.0001)
                    continue;

                ScanBufTemp.x = d_x;

//            ROS_DEBUG("d_x=%f",d_x);

                m_dScanManBufDeque.push_back(ScanBufTemp);
            }
            lock.unlock();
            m_dScanSrcBufDeque.clear();
            m_ScanBufCondition.notify_one();
            m_dDisTemp = dDistance;
        }
    }
    lms_.StopMeasurement ();
}

/*************************************************
Function: CTrackRobot::ManageScanDataThreadFunc
Description:数据过滤线程功能函数，保证两组数据中x的差值大于0.0001米
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::ManageScanDataThreadFunc()
{
    unsigned long ulSize;
    std::deque<sScanBuf> sSCanBufDeque;

    while(ros::ok())
    {
        std::unique_lock<std::mutex> lock(m_ScanManBufMutex);
        while(m_dScanManBufDeque.empty())
        {
            m_ScanBufCondition.wait(lock);
        }

        sSCanBufDeque.assign(m_dScanManBufDeque.begin(), m_dScanManBufDeque.end());
        m_dScanManBufDeque.clear();
        lock.unlock();

        if(!m_bMotorRunning)
        {
            sSCanBufDeque.clear();
            continue;
        }

        ulSize = sSCanBufDeque.size();
        double xTemp = 0;
        sPositionData positionData = {0};
        int nDataLen = 0;
        for (int i = 0; i < ulSize; i++)
        {
            sScanBuf ScanBufTemp = sSCanBufDeque[i];

            if(i == 0 || (abs(ScanBufTemp.x - xTemp) >= 0.0001))
            {
                xTemp = ScanBufTemp.x;
                if(xTemp <= m_dTargetDis && xTemp >= 0)
                {
                    nDataLen = TransPosBufData(positionData, ScanBufTemp);
                    OutputPosData(positionData, nDataLen);
                }
            }
        }
        sSCanBufDeque.clear();
    }
}

/*************************************************
Function: CTrackRobot::ManagePosBufData
Description:将激光原始数据转换成对应的侵限值y以及高度z
Input: sPositionData &desPosData 存放处理后的数据， x y[] z[]
       sScanBuf srcBufData      激光原始buf数据
Output: int, y z 数据有效值的数据长度
Others: void
**************************************************/
int CTrackRobot::TransPosBufData(sPositionData &desPosData, const sScanBuf &srcBufData, bool bAbsolute)
{
    double d_y, d_z;
    int nValidSize = 0;

    //里程x值，侵限点定位情况为绝对定位，所以需要单独计算x值。
    if(!bAbsolute && m_nControlMode != ALARM)
    {
        desPosData.x =  srcBufData.x;
        if(m_nDirection == BACKWARD)
        {
            desPosData.x = m_dTargetDis - desPosData.x;
        }
    }
    else
    {
        double dDistance;
        int nRobotDisplacement;

        if(m_nMode == PP)
        {
            nRobotDisplacement = m_nRobotCurrentPulseTwo - m_nRobotOriginPulseTwo;
        }
        else
        {
            nRobotDisplacement = (m_nRobotCurrentPulseOne - m_nRobotOriginPulseOne)/2 + (m_nRobotCurrentPulseTwo - m_nRobotOriginPulseTwo)/2;
        }

        dDistance = (nRobotDisplacement/(PULSES_OF_MOTOR_ONE_TURN*MOTOR_TRANSMISSION))*M_PI*forward_diameter/1000;

        if(!m_bForwardRelocation)
        {
//            ROS_DEBUG("[TransPosBufData] m_dRealDis:%f, dDistance:%f", m_dRealDis, dDistance);
            dDistance = m_dRealDis + dDistance;
        }
        desPosData.x = dDistance;
    }

    double dInitialAngle = m_dInitialAngle, dPitchAngle = -m_dPitchAngle;
    double d_dy = HALF_OF_TRACK_DISTANCE - LASER_TRACK_POINT_DISTANCE*cos((LASER_TRACK_POINT_ANGLE - dPitchAngle)*M_PI/180);
    double d_dz = LASER_TRACK_POINT_DISTANCE*sin((LASER_TRACK_POINT_ANGLE - dPitchAngle)*M_PI/180);
//    ROS_DEBUG("[dInitialAngle]ddy=%f,ddz=%f",d_dy, d_dz);

//    ROS_INFO("[dInitialAngle]dInitialAngle=%f",dInitialAngle);
    for(int i=0; i< srcBufData.ullRangeSize; i++)
    {
        if(srcBufData.fRanges[i] < 0.1)
            continue;
        float fCalcRadian = (dInitialAngle-LASER_ANGLE_INCREMENT*i)*M_PI/180;
        d_y = srcBufData.fRanges[i]*sin(fCalcRadian);
        d_y += d_dy;
        d_z = srcBufData.fRanges[i]*cos(fCalcRadian);
        d_z += d_dz;

        if(d_z < 0.35 || d_y < LASER_TRACK_CENTER_DISTANCE + 0.5)
            continue;

        desPosData.y[nValidSize] = (float)d_y;
        desPosData.z[nValidSize] = (float)d_z;
        if(1 == m_nRecordLaser)
        {
            desPosData.laser_dis[nValidSize] = srcBufData.fRanges[i];
            desPosData.laser_angle[nValidSize] = fCalcRadian;
        }
        nValidSize++;
    }

    return nValidSize;
}

/*************************************************
Function: CTrackRobot::OutputPosData
Description: 发布位置数据:x, y[], z[], 并保存至文件大当中
Input: sPositionData &desPosData , 位置数据结构体
       int nDataLen, y z数组对应的有效数据的长度
       int nWriteDataToFile , 是否将位置数据写入到文件当中
Output: void
Others: void
**************************************************/
void CTrackRobot::OutputPosData(sPositionData &desPosData, int nDataLen)
{
    custom_msgs::TrainRobotPosition pos;

    pos.y.resize((unsigned long)nDataLen);
    pos.z.resize((unsigned long)nDataLen);
    if(1 == m_nRecordLaser)
    {
        pos.laser_dis.resize((unsigned long)nDataLen);
        pos.laser_angle.resize((unsigned long)nDataLen);
    }

    pos.x = desPosData.x;

    for(int i = 0; i < nDataLen; i++)
    {
        pos.y[i] = desPosData.y[i];
        pos.z[i] = desPosData.z[i];
        if(1 == m_nRecordLaser)
        {
            pos.laser_dis[i] = desPosData.laser_dis[i];
            pos.laser_angle[i] = desPosData.laser_angle[i];
        }
    }
    pos.trans_id = 0;

    pos.header.stamp = ros::Time::now();
    pos.header.frame_id = "position";
    m_PositionPublisher.publish(pos);
}

/*************************************************
Function: CTrackRobot::UpdateOriginPosition
Description:更新机器人原点位置值
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::UpdateOriginPosition()
{
    //获取run指令下达时的位置值,并将其置为里程原点值
    SendGetMotorInfoCmd();
    this_thread::sleep_for(std::chrono::milliseconds(20));

    m_DynamicOrigin.nMotorOnePos = m_CurrentMotorPos.nMotorOnePos;
    m_DynamicOrigin.nMotorTwoPos = m_CurrentMotorPos.nMotorTwoPos;

    if(m_bManagerUpdateOrigin)
    {
        m_bManagerUpdateOrigin = false;
        m_nRobotOriginPulseOne = m_nRobotCurrentPulseOne;
        m_nRobotOriginPulseTwo = m_nRobotCurrentPulseTwo;
    }
    m_dDistance = 0.0;
    m_bIsUpdateOrigin = true;
    ROS_DEBUG("[UpdateOriginPosition] m_DynamicOrigin.nMotorOnePos=%d,m_DynamicOrigin.nMotorTwoPos=%d",m_DynamicOrigin.nMotorOnePos,m_DynamicOrigin.nMotorTwoPos);
}

/*************************************************
Function: CTrackRobot::WaitForMotionStop
Description: 等待机器人运动停止
Input: int nTimeOutLen, 超时时长
Output: true or false
Others: void
**************************************************/
bool CTrackRobot::WaitForMotionStop(int nTimeOutLen)
{
    timespec StartTime, CurrentTime;
    long pollInterval;
    timespec_get(&StartTime, TIME_UTC);
    while(m_bMotorRunning)
    {
        timespec_get(&CurrentTime, TIME_UTC);
        pollInterval=((CurrentTime.tv_sec - StartTime.tv_sec)\
                                 + (CurrentTime.tv_nsec - StartTime.tv_nsec)) / 1000000000;
        if(pollInterval > nTimeOutLen)
        {
            ROS_ERROR("[WaitMotionDone] wait for done timeout error,pollInterval=%ld",pollInterval);
            return false;
        }
        this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    return true;
}

/*************************************************
Function: CTrackRobot::CheckJoyStatus
Description:检测手柄消息是否发送结束，检测伺服是否上报报文，伺服重启后给伺服初始化
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::CheckJoyStatus()
{
    timespec CurrentTime;
    long pollInterval;

    //虚拟手柄控制超时检查
    if(m_nControlMode == JOY)
    {
        std::unique_lock<std::mutex> lock(m_JoyTimeOutCheckMutex);
        if(m_tTimeOfLastJoyCmd.tv_sec != 0 && m_tTimeOfLastJoyCmd.tv_nsec != 0)
        {
            timespec_get(&CurrentTime, TIME_UTC);
            pollInterval=((CurrentTime.tv_sec - m_tTimeOfLastJoyCmd.tv_sec) * 1000000000 \
                             + (CurrentTime.tv_nsec - m_tTimeOfLastJoyCmd.tv_nsec)) / 1000000;
            if(pollInterval > m_nJoyCtrlTimeoutMSec)
            {
                m_tTimeOfLastJoyCmd.tv_sec = 0;
                m_tTimeOfLastJoyCmd.tv_nsec = 0;
                ROS_DEBUG("[CheckJoyStatus] joy time out, pollInterval=%ld",pollInterval);

                if (abs(m_dTargetSpeed) <= 0.2)
                {
                    QuickStop();
                }
                else
                {
                    StopMotion(true);
                }

                m_nControlMode = IDLE;
            }
        }
        lock.unlock();
    }
}

/*************************************************
Function: CTrackRobot::CheckCanStatus
Description:检测手柄消息是否发送结束，检测伺服是否上报报文，伺服重启后给伺服初始化
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::CheckCanStatus()
{
    timespec CurrentTime;
    long pollInterval;
    //伺服报文反馈超时监测
    if(m_bIsMotorInit)
    {
        if(m_tMotorOneLastRespTime.tv_sec != 0 && m_tMotorOneLastRespTime.tv_nsec != 0)
        {
            timespec_get(&CurrentTime, TIME_UTC);
            pollInterval=((CurrentTime.tv_sec - m_tMotorOneLastRespTime.tv_sec) * 1000000000 \
                             + (CurrentTime.tv_nsec - m_tMotorOneLastRespTime.tv_nsec)) / 1000000;
            if(pollInterval > 2000)
            {
                m_tMotorOneLastRespTime.tv_sec = 0;
                m_tMotorOneLastRespTime.tv_nsec = 0;
                m_bMotorRunning = false;
                m_bIsMotorInit = false;
                StopMotion();

                ROS_INFO("[CheckJoyStatus] motor one response timeout,pollInterval=%ld",pollInterval);
                m_sMotorOneStatus = "TimeOutErr";
            }
        }
        if(m_tMotorTwoLastRespTime.tv_sec != 0 && m_tMotorTwoLastRespTime.tv_nsec != 0)
        {
            timespec_get(&CurrentTime, TIME_UTC);
            pollInterval=((CurrentTime.tv_sec - m_tMotorTwoLastRespTime.tv_sec) * 1000000000 \
                             + (CurrentTime.tv_nsec - m_tMotorTwoLastRespTime.tv_nsec)) / 1000000;
            if(pollInterval > 2000)
            {
                m_tMotorTwoLastRespTime.tv_sec = 0;
                m_tMotorTwoLastRespTime.tv_nsec = 0;
                m_bMotorRunning = false;
                m_bIsMotorInit = false;
                StopMotion();

                ROS_INFO("[CheckJoyStatus] motor two response timeout,pollInterval=%ld",pollInterval);
                m_sMotorTwoStatus = "TimeOutErr";
            }
        }
    }
}

/*************************************************
Function: CTrackRobot::CheckJoyStatus
Description:检测手柄消息是否发送结束，检测伺服是否上报报文，伺服重启后给伺服初始化
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::CheckEStopStatus()
{
    bool bEStopStatus = m_bIsEStopButtonPressed;
    //监测急停是否按下
    m_bIsEStopButtonPressed = (GpioControl(GET_EMERGENCY_STOP_BUTTON_STATUS) == 0);
    //急停由按下切换至放开时进行电机伺服初始化
    if(m_bIsEStopButtonPressed != bEStopStatus && bEStopStatus)
    {
        ROS_DEBUG("[CheckEStopStatus] E-Stop is up, call MotorServoInit() ");
        MotorServoInit();
    }
}

/*************************************************
Function: CTrackRobot::CheckStatusThreadFunc
Description:检测手柄消息是否发送结束，检测伺服是否上报报文，伺服重启后给伺服初始化
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::HeartBeatThreadFunc()
{
    ROS_INFO("[HeartBeatThreadFunc] start");
    custom_msgs::TrainRobotHeartBeat heartBeatMsg;
    heartBeatMsg.sender = "track_robot";
    heartBeatMsg.temperature.resize(3);

    while(ros::ok())
    {
        SendCmdOfGetBatteryInfo();

        if(m_bIsMotorInit)
        {
            SendCmdOfGetMotorErrorCode();
            SendHeartBeatCanFrame();
        }
        this_thread::sleep_for(std::chrono::milliseconds(20));

        heartBeatMsg.status = 0x00;
        heartBeatMsg.status |= m_bIsEStopButtonPressed ? 0x01:0x00;
        heartBeatMsg.status |= m_nBatteryStatus != 0 ? 0x02:0x00;
        heartBeatMsg.status |= !m_bIsEStopButtonPressed && (m_sMotorOneStatus != "normal" || m_sMotorTwoStatus != "normal") ? 0x04:0x00;
        heartBeatMsg.status |= (m_nBatteryCharge == 1) ? 0x08:0x00;

        heartBeatMsg.velocity_x = m_dCurrentSpeed;

        double dDistance;
        int nRobotDisplacement;

        if(m_nMode == PP)
        {
            nRobotDisplacement = m_nRobotCurrentPulseTwo - m_nRobotOriginPulseTwo;
        }
        else
        {
            nRobotDisplacement = (m_nRobotCurrentPulseOne - m_nRobotOriginPulseOne)/2 + (m_nRobotCurrentPulseTwo - m_nRobotOriginPulseTwo)/2;
        }

        dDistance = (nRobotDisplacement/(PULSES_OF_MOTOR_ONE_TURN*MOTOR_TRANSMISSION))*M_PI*forward_diameter/1000;

        if(!m_bForwardRelocation)
        {
            dDistance = m_dRealDis + dDistance;
        }
        heartBeatMsg.position_x = dDistance;

        heartBeatMsg.light_status = m_nLightStatus;
        heartBeatMsg.battery_voltage = (m_nBatteryVoltage/10.0);
        heartBeatMsg.battery_quantity = m_nBatteryPower;
        heartBeatMsg.battery_status = m_nBatteryStatus;

        heartBeatMsg.motor_status = "normal";
        if(m_sMotorOneStatus != "normal" && !m_bIsEStopButtonPressed)
            heartBeatMsg.motor_status = m_sMotorOneStatus;
        if(m_sMotorTwoStatus != "normal" && !m_bIsEStopButtonPressed)
            heartBeatMsg.motor_status += m_sMotorTwoStatus;

        heartBeatMsg.temperature[0] = m_nBatteryTemp;
        heartBeatMsg.temperature[1] = m_nMotorOneTemp;
        heartBeatMsg.temperature[2] = m_nMotorTwoTemp;

        heartBeatMsg.pitch_angle = m_dPitchAngle;

        heartBeatMsg.header.stamp = ros::Time::now();

        m_HeartBeatPublisher.publish(heartBeatMsg);
        this_thread::sleep_for(std::chrono::milliseconds(980));
    }
}

/*************************************************
Function: CTrackRobot::RestartMotorServo
Description:重启伺服驱动器
Input: void
Output: int
Others: void
**************************************************/
int CTrackRobot::RestartMotorServo()
{
    if(GpioControl(POWER_OFF) == -1)
    {
        ROS_ERROR("[RestartMotorServo] turn off power electric failed");
        return -1;
    }
    this_thread::sleep_for(std::chrono::milliseconds(500));
    if(GpioControl(LOGIC_OFF) == -1)
    {
        ROS_ERROR("[RestartMotorServo] turn off logic electric failed");
        return -1;
    }

    this_thread::sleep_for(std::chrono::milliseconds(1000));

    if(GpioControl(LOGIC_ON) == -1)
    {
        ROS_ERROR("[RestartMotorServo] turn on logic electric failed");
        return -1;
    }

    this_thread::sleep_for(std::chrono::milliseconds(500));
    if(GpioControl(POWER_ON) == -1)
    {
        ROS_ERROR("[RestartMotorServo] turn on power electric failed");
        return -1;
    }

    m_sMotorOneStatus = "normal";
    m_sMotorTwoStatus = "normal";
    return 1;
}

/*************************************************
Function: CTrackRobot::RestartMotorServo
Description:重启伺服驱动器
Input: void
Output: void
Others:
	GPIO名称	    GPIO功能定义            逻辑
输入DI:
	GPIO1_A0	急停按钮信号输入	急停复位-1；急停按下-0
	GPIO1_D0	备用输入（24V）
输出DO:
	GPIO1_A1	驱动器逻辑供电     1-逻辑供电；0-逻辑断电
	GPIO1_A3	灯带B输出	        1-蓝色亮；0-蓝色熄灭
	GPIO1_A4	灯带G输出	        1-绿色亮；0-绿色熄灭
	GPIO1_C6	灯带R输出	        1-红色亮；0-红色熄灭
	GPIO1_C7	驱动器动力供电	    1-动力供电；0-动力断电
	GPIO1_C2	备用输出（24V）
**************************************************/
int CTrackRobot::GpioControl(int nCtrlWord)
{
    CGpioControl GpioCtrl;
    string sDirection = "out";
    int nGpioNum = 0;
    int nGpioValue = 0;

    if(nCtrlWord == GET_EMERGENCY_STOP_BUTTON_STATUS)
    {
        sDirection = "in";
        nGpioNum = GPIO1_A0;
        GpioCtrl.SetPinAndDirection(nGpioNum, sDirection);

        return GpioCtrl.GetGpioValue();
    }

    switch(nCtrlWord)
    {
        case LOGIC_ON:
            nGpioValue = HIGH; nGpioNum = GPIO1_A1;
            break;
        case LOGIC_OFF:
            nGpioValue = LOW; nGpioNum = GPIO1_A1;
            break;
        case R_LIGHT_ON:
            nGpioValue = HIGH; nGpioNum = GPIO1_C6; m_nLightStatus = 0;
            break;
        case R_LIGHT_OFF:
            nGpioValue = LOW; nGpioNum = GPIO1_C6;
            break;
        case G_LIGHT_ON:
            nGpioValue = HIGH; nGpioNum = GPIO1_A4; m_nLightStatus = 1;
            break;
        case G_LIGHT_OFF:
            nGpioValue = LOW; nGpioNum = GPIO1_A4;
            break;
        case B_LIGHT_ON:
            nGpioValue = HIGH; nGpioNum = GPIO1_A3; m_nLightStatus = 2;
            break;
        case B_LIGHT_OFF:
            nGpioValue = LOW; nGpioNum = GPIO1_A3;
            break;
        case POWER_ON:
            nGpioValue = HIGH; nGpioNum = GPIO1_C7;
            break;
        case POWER_OFF:
            nGpioValue = LOW; nGpioNum = GPIO1_C7;
            break;
        default:
            ROS_WARN("[GpioControl] ctrl word error");
            return -1;
    }

    timespec StartTime;
    timespec_get(&StartTime, TIME_UTC);

    GpioCtrl.SetPinAndDirection(nGpioNum, sDirection);
    while(GpioCtrl.GetGpioValue() != nGpioValue)
    {
        GpioCtrl.SetGpioValue(HIGH);
        this_thread::sleep_for(std::chrono::milliseconds(10));

        timespec CurrentTime;
        timespec_get(&CurrentTime, TIME_UTC);

        long pollInterval=((CurrentTime.tv_sec - StartTime.tv_sec) * 1000000000 \
                                 + (CurrentTime.tv_nsec - StartTime.tv_nsec)) / 1000000;
        if(pollInterval > 100)
        {
            ROS_ERROR("[GpioControl] set value timeout error");
            return -1;
        }
    }
    return 1;
}

CTrackRobot::~CTrackRobot()
{
    ROS_DEBUG("[CTrackRobot][~CTrackRobot] begin...");
    UsbCan.Close();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    m_pCANReceiveThread->join();
#if USE_CAN2_RT
    m_pCAN2ReceiveThread->join();
#endif
    m_pCANManageThread->join();
    m_pCANSendThread->join();
    m_pControlThread->join();
    m_pAutoRunThread->join();
    m_pPositionFilterThread->join();
    m_pCheckStatusThread->join();
    m_pHeartBeatThread->join();
    m_pScanThread->join();

    delete m_pCANReceiveThread;
#if USE_CAN2_RT
    delete m_pCAN2ReceiveThread;
#endif
    delete m_pCANManageThread;
    delete m_pCANSendThread;
    delete m_pControlThread;
    delete m_pAutoRunThread;
    delete m_pPositionFilterThread;
    delete m_pCheckStatusThread;
    delete m_pHeartBeatThread;
    delete m_pScanThread;
}

/*************************************************
Function: CTrackRobot::updata_XML
Description:修改配置文件中的轮子直径
Input:
    path 配置文件名
    updataposion，从外到更新节点的路径
    attributes,更新元素

Output: 执行结果
**************************************************/
int CTrackRobot::updata_XML(string paramName, string paramValue)
{
    XMLDocument doc;
    char buf[100];
    getcwd(buf,100);
    strcat(buf,"/../catkin_ws/src/track_robot/launch/track_robot.launch");
    const char* path_updata = buf;
    const char* paranName_updata = paramName.c_str();
    const char* paramValue_updata = paramValue.c_str();
    if (doc.LoadFile(path_updata))
    {
        ROS_DEBUG("the launch file is:%s",buf);
        return 0;
    }
    XMLElement *current_root = doc.RootElement();
    XMLElement *nodeName = current_root->FirstChildElement("node");
    if(NULL==nodeName)
        return 0;
    XMLElement *paramNode = nodeName->FirstChildElement("param");
    while (paramNode!=NULL)
    {
        string str = paramNode->Attribute("name");
        if(str.compare(paranName_updata)==0)
        {
            paramNode->SetAttribute("value",paramValue_updata);
            break;
        }
        paramNode = paramNode->NextSiblingElement();
    }

    doc.SaveFile(path_updata, false);
    return 1;
}

//added by btrmg for filter 2020.04.13
/***********************************************************
Function: CTrackRobot::FilterPoints
Description: 根据输入直线的y,z值,输出过滤后的y,z数组
Input: y[] z[]
Output: y[] z[]
Others: void
************************************************************/
void CTrackRobot::FilterPoints(vector<float> in_y,vector<float > in_z,vector<double> &out_y,vector<double > &out_z)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = in_y.size();
    cloud->height = 1;
    cloud->points.resize(cloud->width*cloud->height);
    string re_str;
//    int point_number = cloud->points.size();
//    ROS_INFO("the point size is:%d",point_number);
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        cloud->points[i].x = 1.0;
        cloud->points[i].y = in_y[i];
        cloud->points[i].z = in_z[i];
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr FirstFilterCloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;  //创建滤波器

    radiusoutlier.setInputCloud(cloud);    //设置输入点云
    radiusoutlier.setRadiusSearch(m_d1stFilterRadius);     //设置半径为5cm的范围内找临近点
    radiusoutlier.setMinNeighborsInRadius(m_n1stFilterNum); //设置查询点的邻域点集数小于5的删除
    radiusoutlier.filter(*FirstFilterCloud);
//    if(cloud_after_Radius->points.size() != cloud->points.size())
//    {
//        int filtered_point = cloud->points.size()-cloud_after_Radius->points.size();
//        ROS_INFO("filter %d points",filtered_point);
//    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr SecondFilterCloud(new pcl::PointCloud<pcl::PointXYZ>);
    radiusoutlier.setInputCloud(FirstFilterCloud);    //设置输入点云
    radiusoutlier.setRadiusSearch(m_d2ndFilterRadius);     //设置半径为1cm的范围内找临近点
    radiusoutlier.setMinNeighborsInRadius(m_n2ndFilterNum); //设置查询点的邻域点集数小于3的删除
    radiusoutlier.filter(*SecondFilterCloud);

    out_y.resize(SecondFilterCloud->points.size());
    out_z.resize(SecondFilterCloud->points.size());
    for(size_t k=0;k<SecondFilterCloud->points.size();k++)
    {
        out_y[k] =  SecondFilterCloud->points[k].y;
        out_z[k] = SecondFilterCloud->points[k].z;
    }
}
