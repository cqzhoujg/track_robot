前端控制消息 TrainRobotControl
Header      header
string      sender
string      receiver
string      robot_ip
uint32      trans_id
string       cmd        # run, stop, start, return
uint32       back       # 1, 0
float64     speed       # 正 或者 负 （单位m/s） 决定机器人向前或者向后运动
float64     distance    # 目标距离 （单位m）
float64     limit_dis   # 侵限阀值 （单位mm）


功能列表

1.运动模式
    ①模拟手柄模式      JOY
        固定低速运动，实现小位置挪动，手柄持续按下时运动，松开则停止运动
    ②任务模式         TASK
        按照前台下发的目标距离及速度运动
    ③侵限点定位模式    ALARM_LOCATION
        按照前台下发的目标距离及速度运动，定位到目标距离前后 LIMIT_LOCATION_RANGE_PARAM 厘米的侵限点，详细将下文 8。

2.机器人运动：
    前进              FORWARD
    后退              BACKWARD
    返回              RETURN：
         前进对应的返回      FORWARD_RETURN
         后退对应的返回      BACKWARD_RETURN
    自动返回            由 m_pAutoRunThread 线程实现，当前台运动指令（前进 or 后退）中back参数为1时，运动结束后，返回指令发起时的位置

    停止              STOP
    恢复              START

    匀加速及匀减速使用的伺服的匀加速和匀减速，前进后退是通过速度的正负来实现，正速度向前，负速度向后

3.里程计算
    通过指令查询伺服编码器的实时绝对位置值来计算里程。统计电机转了多少转，并通过轮子直径换算出具体的距离值。
    每20个ms查询一次伺服编码器的位置，计算更新里程x的值

4.激光数据处理
    激光数据来自于激光节点laser_node，数据发送间隔周期大概在9-10个ms，频率大于里程x的频率，所以基于激光的频率对x做品均差值，
    将插值后的x及其对应的激光原始数据保存到buff当中，由 m_pPositionFilterThread 线程处理buff中的数据：换算y z，并将xyz数据保存至文件的同时
    封装成ros消息后通过m_PositionPublisher发布出去。

5.位置信息写入文件
    前台每一次指令，都会保存一组数据，为了规避频繁open close同一个文件
    （频繁open close的话，当文件中的数据量较大时，将会占据很大的时间片，导致 程序效率低下），在每次收到前台的前进或者后退指令时，
    创建并打开一个以当前时间命名的txt文件，当运动结束时在close该文件（m_pAutoRunThread中close）


6.位置消息发布
    位置消息分为两种，1.当前实时数据（机器人静止状态，运动状态下，该功能无效），对应的消息 trans_id 为100
                    2.运动过程中的实时数据，对应的消息 trans_id 为0
    区分1和2控制参数为 m_bGetAxesData，由前台指令置位。

7.目标距离定位
    理论上目标距离的换算方式为：
    s=(1/2 * a * t^2) + V0*t0 + (1/2 * a * t^2) = V0*t0 + a * t^2,

    由于伺服的加速度存在一定的误差，且存在通讯延时，顾上面的计算方式会存在不确定误差，顾采用如下方式实现定位：
    ①当目标距离 s >= (a * t^2 + END_DIS_AMEND)时：
        s=V0*t1 + a * t^2 + END_DIS_AMEND,
        END_DIS_AMEND为引入值，可自由配置，当实时距离 s1 = s - ((1/2 * a * t^2) + END_DIS_AMEND)时,下达匀减速指令使之减速至0.1m/s，
        后以0.1m/s的速度做匀速运动，由另外一个线程实时检测当前的距离，当实时距离到达目标距离时发送快速停止指令。
    ②当目标距离 s < (a * t^2 + END_DIS_AMEND)时：
        当实时距离到达 s1 = s * END_DIS_MODIFICATION时(END_DIS_MODIFICATION同样为配置参数，目前设置为0.55)，
        下达匀减速指令使之减速至0.1m/s，后以0.1m/s的速度做匀速运动，由另外一个线程实时检测当前的距离，当实时距离到达目标距离时发送快速停止指令。

8.侵限点定位
    按照前台下发的目标距离及速度运动，定位到目标距离前后 LIMIT_LOCATION_RANGE_PARAM 距离的侵限点。停止指令又两处发送，并发监测执行
    (1)AutoRun线程实现定位：
     ①当目标距离 s >= (a * t^2 + END_DIS_AMEND)时：
       s=V0*t1 + a * t^2 + END_DIS_AMEND,
       END_DIS_AMEND为引入值，可自由配置，当实时距离 s1 = s - ((1/2 * a * t^2) + END_DIS_AMEND)时,下达匀减速指令使之减速至0.1m/s，
       后以0.1m/s的速度做匀速运动，当时实时距离 s1 >= s + LIMIT_LOCATION_RANGE_PARAM时，发送快速停止指令。
     ②当目标距离 s < (a * t^2 + END_DIS_AMEND)时：
       当实时距离到达 s1 = s * END_DIS_MODIFICATION时(END_DIS_MODIFICATION同样为配置参数，目前设置为0.55)，
       下达匀减速指令使之减速至0.1m/s，后以0.1m/s的速度做匀速运动，当时实时距离s1 >= s + LIMIT_LOCATION_RANGE_PARAM时，发送快速停止指令。
    (2)数据处理线程定位：
      当时实时距离s1 >= s - LIMIT_LOCATION_RANGE_PARAM时，由侵限值换算线程中判定当前位置的数据中是否有侵限值，若有则发送快速停止指令

    注意：(1)(2)是并发进行的

------------------------------------------华丽的分割线------------------------------------------------
检测电池故障信息：
    主机下发电池
    数据	DATA[8]= 16 BE XX XX XX XX XX 7E
    电池上传主机
    数据	DATA[8]
    字节1	VSTATE 高位
    字节2	VSTATE 低位
    字节3	CSTATE 高位
    字节4	CSTATE 低位
    字节5	TSTATE 高位
    字节6	TSTATE 低位
    字节7	FETSTATE
    字节8	Alarm
struct _VSTATE_
{
    uint16_t VOV:1;//单体过压
    uint16_t VUV:1;//单体欠压
    uint16_t BVOV:1;//电池组过压
    uint16_t BVUV:1;//电池组欠压
    uint16_t wVOV:1;//单体过压警告值
    uint16_t wVUV:1;//单体过压警告值
    uint16_t wBVOV:1;//电池组过压警告值
    uint16_t wBVUV:1;//电池组欠压警告值

    uint16_t VDIFF:1;//压差保护
    uint16_t VBREAK:1;//断线
    uint16_t CSGDIS:1;//低压禁止充电
};

struct _CSTATE_
{
    uint16_t CING:1;//充电状态
    uint16_t DING:1;//放电状态
    uint16_t OCCSG:1;//充电过流
    uint16_t SHORT:1;//短路保护
    uint16_t OCDSG1:1;//放电过流1级
    uint16_t OCDSG2:1;//放电过流2级
    uint16_t wOCCSG:1;//充电电流警告值
    uint16_t wOCDSG:1;//放电电流警告值
};

struct _TSTATE_
{
    uint16_t TCELL_CSGH:1;//充电高温
    uint16_t TCELL_CSGL:1;//充电低温
    uint16_t TCELL_DSGH:1;//放电高温
    uint16_t TCELL_DSGL:1;//放电低温
    uint16_t TENV_H:1;//环境低温
    uint16_t TENV_L:1;//环境高温
    uint16_t TFET_H:1;//功率低温
    uint16_t TFET_L:1;//功率高温

    uint16_t wTCELL_H:1;//电芯高温警告
    uint16_t wTCELL_L:1;//电芯低温警告
    uint16_t wTENV_H:1;//环境高温警告
    uint16_t wTENV_L:1;//环境低温警告
    uint16_t wTFET_H:1;//功率高温警告
    uint16_t wTFET_L:1;//功率低温警告
};

struct _FETSTATE_
{
    uint8_t DFET:1;//放电开关状态，1为打开，0为关闭
    uint8_t CFET:1;//充电开关状态，1为打开，0为关闭
    uint8_t SDFET:1;//放电开关，1为打开，0为关闭
    uint8_t SCFET:1;//充电开关，1为打开，0为关闭

    uint8_t DFET_DAMAGE:1;//放电MOS标志，1为损坏
    uint8_t CFET_DAMAGE:1;//充电MOS标志，1为损坏
    uint8_t CCFET:1;//预留位，1，ON
};

struct _ALARM_
{
    uint8_t bit0:1;//电压报警，压差保护，断线保护
    uint8_t bit1:1;//充电fet损坏报警
    uint8_t bit2:1;//SD ERR 1 error,0 normal
    uint8_t SPI_ERR:1;//ML5238通信

    uint8_t E2PROM_ERR:1;//外部存储E2PROM ERR
    uint8_t bit5:1;//保留
    uint8_t FCC_UPDATING:1;//充电学习开启状态
    uint8_t FCC_DSGLEARN:1;//放电学习开启状态
};