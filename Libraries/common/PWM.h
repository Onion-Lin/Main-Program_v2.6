/*************************************************
Copyright (C), 2016-2023, TYUT JBD TRoMaC
File name: PWM.c
Author: Li_Jiang
Version: v1.0       
Date: 2023_09_16
Description: 对PWM输出功能做简单的封装
Others: NULL
Function List:  PWM_SetDuty()
                PWM_Init()
History:
<author>    <time>          <version>       <desc>
Li_Jiang    2023_09_16      v1.0            对PWM输出功能做简单的封装
**************************************************/
#ifndef __PWM_h
#define __PWM_h

#include "headfiles.h"

/* =========== 宏定义 ================ */

// PWM通道宏
    // TIM1_CH1N--->B13
    // TIM1_CH2N--->B14
    // TIM1_CH3N--->B15
    #define PWM_TIM1_CH1N_B13       ((uint32_t *)&TIM1->CCR1)
    #define PWM_TIM1_CH2N_B14       ((uint32_t *)&TIM1->CCR2)
    #define PWM_TIM1_CH3N_B15       ((uint32_t *)&TIM1->CCR3)
    #define PWM_TIM1_CH4_A11        ((uint32_t *)&TIM1->CCR4)
    
    // TIM2_CH1---->A0
    // TIM2_CH2---->A1
    #define PWM_TIM2_CH1_A0         ((uint32_t *)&TIM2->CCR1)
    #define PWM_TIM2_CH2_A1         ((uint32_t *)&TIM2->CCR2)
    
    // TIM3_CH1---->A6
    // TIM3_CH2---->A7
    // TIM3_CH3---->B0
    #define PWM_TIM3_CH1_A6         ((uint32_t *)&TIM3->CCR1)
    #define PWM_TIM3_CH2_A7         ((uint32_t *)&TIM3->CCR2)
    #define PWM_TIM3_CH3_B0         ((uint32_t *)&TIM3->CCR3)
    
    // TIM4_CH1---->B6
    // TIM4_CH2---->B7
    // TIM4_CH3---->B8
    // TIM4_CH4---->B9
    #define PWM_TIM4_CH1_B6         ((uint32_t *)&TIM4->CCR1)
    #define PWM_TIM4_CH2_B7         ((uint32_t *)&TIM4->CCR2)
    #define PWM_TIM4_CH3_B8         ((uint32_t *)&TIM4->CCR3)
    #define PWM_TIM4_CH4_B9         ((uint32_t *)&TIM4->CCR4)
    
    // TIM5_CH3---->A2
    // TIM5_CH4---->A3
    #define PWM_TIM5_CH3_A2         ((uint32_t *)&TIM5->CCR3)
    #define PWM_TIM5_CH4_A3         ((uint32_t *)&TIM5->CCR4)
    
    // TIM8_CH1---->C6
    #define PWM_TIM8_CH1_C6         ((uint32_t *)&TIM8->CCR1)



/* =========== 全局变量声明 ========== */

/* =========== 函数声明 ============== */
void PWM_Init(void);
void PWM_SetDuty(uint32_t *Channel, int32_t Duty);
int16_t PWM_BreathDuty(void);
#endif /* __PWM_h */

