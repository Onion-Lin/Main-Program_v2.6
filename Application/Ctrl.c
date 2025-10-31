/*************************************************
Copyright (C), 2016-2023, TYUT JBD TRoMaC
File name: Ctrl.c
Author: Li_Jiang
Version: v2.0    
Date: 2023_09_16
Description: 
Others: NULL
Function List: 
History:
<author>    <time>          <version>       <desc>
Li_Jiang    2023_09_10      v1.0            对拨码开关进行简单的封装
**************************************************/
#include "Ctrl.h"

/* ============ 全局变量声明 ============= */
hCtrl_TypeDef hCtrl;

/* ============ 局部变量声明 ============= */

/* ============ 内部函数声明 ============= */

/* ============ 函数定义 ================= */

/**
  * @name   Switch_GetCode
  * @brief  获取拨码开关的键值
  * @call   Internal or External
  * @param  NULL
  * @RetVal Return value
  */
uint8_t Switch_GetCode(void)
{
    // 记录历史值
    hCtrl.Switch.Code = (hCtrl.Switch.Code & 0x55U) << 1;
    
    // 获取本次的值
    // Switch_1
    hCtrl.Switch.Code |= ((Switch_1_GPIO_Port->IDR & Switch_1_Pin) == Switch_Polarity ? 0x01U : 0x00U) << 2 * 0;
    // Switch_2
    hCtrl.Switch.Code |= ((Switch_2_GPIO_Port->IDR & Switch_2_Pin) == Switch_Polarity ? 0x01U : 0x00U) << 2 * 1;
    // Switch_3
    hCtrl.Switch.Code |= ((Switch_3_GPIO_Port->IDR & Switch_3_Pin) == Switch_Polarity ? 0x01U : 0x00U) << 2 * 2;
    // Switch_4
    hCtrl.Switch.Code |= ((Switch_4_GPIO_Port->IDR & Switch_4_Pin) == Switch_Polarity ? 0x01U : 0x00U) << 2 * 3;
    
    return hCtrl.Switch.Code & 0x55U;
    
}/* Switch_GetCode() */



