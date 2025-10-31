/*************************************************
Copyright (C), 2016-2023, TYUT JBD TRoMaC
File name: 
Author: 
Version:               
Date: 
Description:  
Others:      
Function List:
History:
<author>    <time>          <version>       <desc>
**************************************************/
#include "RC.h"

/* ============ 全局变量声明 ============= */
hRC_TypeDef hRC = {0};

/* ============ 局部变量声明 ============= */

/* ============ 内部函数声明 ============= */

/* ============ 函数定义 ================= */
void RC_Init(void)
{
    hRC.GPIO_Port       = PPM_GPIO_Port;        // 外部中断的通道
    hRC.GPIO_Pin        = PPM_Pin;              // 外部中断的引脚号
    hRC.TIM             = htim_RC.Instance;     // 用来记录时间的计时器
    hRC.State           = RC_state_WaitingForSingnal;
    
    for(uint8_t i = 0; i < RC_ChannelIndexNum; i++)
    {
        hRC.ChannelData_Raw[i] = 0;
        
        hRC.ChannelData_Offset[i] = 0;
    }
    
    hRC.CurrentChannelIndex = 0;
    
    hRC.PPM_Duration_Max = 1600;
    hRC.PPM_Duration_Min = 1000;
    
    hRC.TIM->CR1 |= TIM_CR1_CEN;    // 定时器开启
}

void RC_ReceiveCallback(void)
{   
    uint16_t    CurrentDuration = 0;        // 当前信号的持续时长
    
    // 状态切换
    if(hRC.State == RC_state_WaitingForSingnal || hRC.State == RC_state_Ready)
    {
        hRC.State = RC_state_Receiving;
    }
    
    
    // 读取当前信号持续时长
    CurrentDuration = hRC.TIM->CNT;
    hRC.TIM->CNT = 0;
    
    if(hRC.SynchroFlag == 1)
    {
        hRC.SynchroFlag = 0;
        hRC.CurrentChannelIndex = 0;
    }
    
    if(CurrentDuration < 4000 && hRC.CurrentChannelIndex < RC_ChannelIndexNum)
    {
        hRC.ChannelData_Raw[hRC.CurrentChannelIndex] = CurrentDuration;
        hRC.CurrentChannelIndex += 1;
        
        if(hRC.CurrentChannelIndex == RC_ChannelIndexNum)  // 一帧接收完成
        {
            for(uint8_t i = 0; i < RC_ChannelIndexNum; i++)
            {
                hRC.ChannelData_Remap[i] = (hRC.ChannelData_Raw[i] + hRC.ChannelData_Offset[i] - hRC.PPM_Duration_Min) * 2000 / (hRC.PPM_Duration_Max - hRC.PPM_Duration_Min) - 1000;
            }
        }
    }
    else
    {
        hRC.CurrentChannelIndex = 0;
        hRC.SynchroFlag = 1;
    }
         
}

/* ==== PPM原理示意图 ==== */
/* ===================================================================================================================== *
 *          |-t1--|-t2-----|-t3-|-t4----|-t5-------|-t6--|-Synchro Blank Time---------|-t1--|-t2-----|-t3--              *
 *           _     _        _    _       _          _     _                            _     _        _                  *
 * _________/ \___/ \______/ \__/ \_____/ \________/ \___/ \____________---___________/ \___/ \______/ \___ ---- CH_PPM  *
 *          |     |        |    |       |          |     |                            |     |        |                   *
 *          |_____|        |    |       |          |     |                            |_____|        |                   *
 * _________/     \________|____|_______|__________|_____|______________---___________/     \________|_____ ---- CH_1    *
 *                |        |    |       |          |     |                                  |        |                   *
 *                |________|    |       |          |     |                                  |________|                   *
 * _______________/        \____|_______|__________|_____|______________---_________________/        \_____ ---- CH_2    *
 *                         |    |       |          |     |                                           |                   *
 *                         |____|       |          |     |                                           |_____              *
 * ________________________/    \_______|__________|_____|______________---__________________________/      ---- CH_3    *
 *                              |       |          |     |                                                               *
 *                              |_______|          |     |                                                               *
 * _____________________________/       \__________|_____|______________---________________________________ ---- CH_4    *
 *                                      |          |     |                                                               *
 *                                      |__________|     |                                                               *
 * _____________________________________/          \_____|______________---________________________________ ---- CH_5    *
 *                                                 |     |                                                               *
 *                                                 |_____|                                                               *
 * ________________________________________________/     \______________---________________________________ ---- CH_6    *
 *                                                                                                                       *
 * ===================================================================================================================== */
