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
#ifndef __Ctrl_h
#define __Ctrl_h

#include "headfiles.h"

/* =========== 宏定义 ================ */

#define Switch_Polarity                     GPIO_PIN_RESET                                  // Switch有效电平
#define Switch_GetState_Index(Index)        ((hCtrl.Switch.Code >> (2 * (Index - 1))) & 0x01U)      // 根据Index获取单个键位的状态

#define Switch_EN_ON                        ((Switch_EN_GPIO_Port->IDR & Switch_EN_Pin) != 0x00U)
#define Switch_EN_OFF                       (!Switch_EN_ON)


typedef enum
{
    Switch_Index_1              = 1,
    Switch_Index_2              = 2,
    Switch_Index_3              = 3,
    Switch_Index_4              = 4,
}Switch_Index_enum;



typedef struct
{
    struct 
    {
        uint8_t     Code;
    }Switch;
    
    struct
    {
        uint8_t     Code;
    }LED;
    
    
}hCtrl_TypeDef;

/* =========== 全局变量声明 ========== */
extern hCtrl_TypeDef hCtrl;

/* =========== 函数声明 ============== */
uint8_t Switch_GetCode(void);


#endif /* __Ctrl_h */



