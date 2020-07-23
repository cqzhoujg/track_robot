//
// Created by zhoujg on 19-10-22.
//

#ifndef PROJECT_CGPIOCONTROL_H
#define PROJECT_CGPIOCONTROL_H

#include <iostream>
#include <string.h>
#include <mutex>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
/*************************************************
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
typedef enum _tagGpioPinNum_
{
    GPIO1_A0 = 32,//PIN_7
    GPIO1_A1 = 33,//PIN_11
    GPIO1_C2 = 50,//PIN_12
    GPIO1_A3 = 35,//PIN_13
    GPIO1_A4 = 36,//PIN_15
    GPIO1_C6 = 54,//PIN_16
    GPIO1_C7 = 55,//PIN_18
    GPIO1_D0 = 56 //PIN_22
}GpioPinNum;    

class CGpioControl
{
public:
    CGpioControl(){}

    CGpioControl(int nGpioNum, string &sDirection)
    {
        SetGpioPin(nGpioNum);
        SetGpioDirection(sDirection);
    }

    void SetPinAndDirection(int nGpioNum, string &sDirection)
    {
        SetGpioPin(nGpioNum);
        SetGpioDirection(sDirection);
    }

    void SetGpioPin(int nPinNum)
    {
        m_nGpioNum = nPinNum;

        m_sGpioDirectionTarget = "/sys/class/gpio/gpio";
        m_sGpioDirectionTarget += to_string(m_nGpioNum);
        m_sGpioDirectionTarget += "/direction";

        m_sGpioValueTarget = "/sys/class/gpio/gpio";
        m_sGpioValueTarget += to_string(m_nGpioNum);
        m_sGpioValueTarget += "/value";
    }

    void SetGpioDirection(const string &sDirection)
    {
        if(sDirection != "out" && sDirection != "in")
        {
            printf("[CGpioControl]parameter error sDirection=%s\n",sDirection.c_str());
            return;
        }

        FILE *fp;
        m_sDirection = sDirection;
        fp = fopen(m_sGpioDirectionTarget.c_str(),"w");
        if(fp == nullptr)
        {
            printf("[CGpioControl][SetGpioDirection]open %s failed.\n",m_sGpioDirectionTarget.c_str());
            return;
        }

        fprintf(fp, "%s", sDirection.c_str());
        fclose(fp);
    }

    string GetGpioDirection()
    {
        return m_sDirection;
    }

    void SetGpioValue(int nLevel)
    {
        FILE *fp;
        m_nGpioValue = nLevel;
        fp = fopen(m_sGpioValueTarget.c_str(),"w");

        if(fp == nullptr)
        {
            printf("[CGpioControl][SetGpioValue]open %s failed.\n",m_sGpioDirectionTarget.c_str());
            return;
        }
        if(nLevel == 1)
        {
            fprintf(fp,"%d",1);
            fclose(fp);
        }
        else if(nLevel == 0)
        {
            fprintf(fp,"%d",0);
            fclose(fp);
        }
        else
        {
            printf("[CGpioControl][SetGpioValue]parameter error nLevel=%d\n",nLevel);
        }
    }

    int GetGpioValue()
    {
        FILE *fp;
        char buf[8];
        int handle;
        ssize_t bytes ;
        int nRet = 0;

        fp = fopen(m_sGpioValueTarget.c_str(),"r");
        if(fp == nullptr)
        {
            printf("[CGpioControl][GetGpioValue]open %s failed.\n",m_sGpioDirectionTarget.c_str());
            return -1;
        }

        bytes = fread (buf, 1, 8, fp) ;

        fclose(fp);

        if(bytes==-1)
        {
            printf("[CGpioControl][GetGpioValue]ReadFailed.\n");
            return -1;
        }
        else
        {
            nRet = (int)buf[0] - 48;
        }
        return nRet;
    }

private:
    string m_sDirection;
    int m_nGpioNum;
    string m_sGpioDirectionTarget;
    string m_sGpioValueTarget;
    int m_nGpioValue;
};
#endif //PROJECT_CGPIOCONTROL_H
