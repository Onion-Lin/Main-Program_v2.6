/*************************************************
Copyright (C), 2016-2023, TYUT JBD TRoMaC
File name: RC.h
Author: Li_Jiang
Version: v0.1          
Date: 2023年9月8日
Description:  
Others:      
Function List:
History:
<author>    <time>          <version>       <desc>
**************************************************/
#ifndef __RC_h
#define __RC_h

#include "headfiles.h"

/* =========== 宏定义 ================ */
#define RC_ChannelIndexNum          8


typedef enum
{
    RC_state_WaitingForSingnal  = 0,
    RC_state_Receiving          ,
    RC_state_Ready              ,
}RC_state_enum;


typedef struct
{
    GPIO_TypeDef   *GPIO_Port;      // 外部中断的通道
    uint16_t        GPIO_Pin;       // 外部中断的引脚号
    
    TIM_TypeDef    *TIM;            // 用于记录时间的计时器
    
    RC_state_enum   State;          // 接收机状态
    uint8_t         SynchroFlag;    // 同步标志
    
    uint16_t        PPM_Duration_Min;
    uint16_t        PPM_Duration_Max;
    
    uint8_t         CurrentChannelIndex;
    uint16_t        ChannelData_Raw[RC_ChannelIndexNum];
    int16_t         ChannelData_Offset[RC_ChannelIndexNum];
    int16_t         ChannelData_Remap[RC_ChannelIndexNum];
    
    
}hRC_TypeDef;

/* =========== 全局变量声明 ========== */
extern hRC_TypeDef hRC;

/* =========== 函数声明 ============== */
void RC_Init(void);
void RC_ReceiveCallback(void);
    

#endif /* __RC_h */

