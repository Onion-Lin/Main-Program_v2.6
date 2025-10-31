/*************************************************
Copyright (C), 2016-2023, TYUT JBD TRoMaC
File name: PWM.c
Author: Li_Jiang
Version: v1.0       
Date: 2023_09_16
Description: 对PWM输出功能做简单的封装
Others: NULL
Function List:  PWM_Init()
                PWM_SetDuty()
History:
<author>    <time>          <version>       <desc>
Li_Jiang    2023_09_16      v1.0            对PWM输出功能做简单的封装
**************************************************/
#include "PWM.h"

/* ============ 全局变量声明 ============= */

/* ============ 局部变量声明 ============= */

/* ============ 内部函数声明 ============= */

/* ============ 函数定义 ================= */



/**
  * @name   PWM_Init()
  * @brief  开启已经初始化的各个PWM通道
  * @call   External
  * @param  NULL
  * @RetVal NULL
  */
void PWM_Init(void)
{
    /* 使能定时器 */
    // TIM1 驱动

    // 配置输出模式
    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM1->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);

    TIM1->CCMR1 &= ~TIM_CCMR1_OC2M;
    TIM1->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);

    TIM1->CCMR2 &= ~TIM_CCMR2_OC3M;
    TIM1->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2);

    // 开使能
    HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_1);   // TIM1_CH1N--->B13
    HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_2);   // TIM1_CH2N--->B14
    HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_3);   // TIM1_CH3N--->B15
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);     // TIM1_CH4 --->A11

    // TIM4 驱动
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);     // TIM4_CH1---->B6
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);     // TIM4_CH2---->B7
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);     // TIM4_CH3---->B8
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);     // TIM4_CH4---->B9

    // TIM8 驱动
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);     // TIM8_CH1---->C6

    // TIM2 舵机
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);     // TIM2_CH1---->A0
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);     // TIM2_CH2---->A1

    // TIM3 舵机
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);     // TIM3_CH1---->A6
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);     // TIM3_CH2---->A7
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);     // TIM3_CH3---->B0

    // TIM5 舵机
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);     // TIM5_CH3---->A2
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);     // TIM5_CH4---->A3
}/* PWM_Init */

/**
  * @name   PWM_SetDuty()
  * @brief  设置PWM通道的占空比
  * @call   External
  * @param  *Channel    PWM通道对应的比较寄存器地址
  * @param  Duty        PWM通道要设置的占空比
  * @RetVal NULL
  */
void PWM_SetDuty(uint32_t *Channel, int32_t Duty)
{
    Duty = ABS(Duty);
    
    *Channel = Duty;
}/* PWM_SetDuty() */

int16_t PWM_BreathDuty(void)
{
    static uint16_t count_Add = 0;
    static uint16_t prescaler = 2;
    static int16_t duty_Min   = 100;
    static int16_t duty_Max   = 8000;
    static int16_t duty_Load  = (8000 + 100) / 2;
    static int8_t  dir        = 1;
    static int8_t  step       = 100;
    
    count_Add++;
    
    if(count_Add >= prescaler)
    {
        if(duty_Load <= duty_Min || duty_Load >= duty_Max)
        {
            dir = -dir;
        }
        
        duty_Load += dir * step;
        
        count_Add = 0;
    }
    
    return duty_Load;
}/* PWM_BreathDuty() */

