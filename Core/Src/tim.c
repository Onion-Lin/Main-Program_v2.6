/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
 * TIM (Timer) 配置文件
 * 
 * 功能概述：
 * - 配置多个定时器实现小车控制
 * - 实现PWM输出控制电机
 * - 实现编码器输入捕获
 * - 实现系统时基管理
 * 
 * 定时器分配：
 * - TIM1: LED控制与系统同步
 * - TIM2: 左轮编码器输入捕获
 * - TIM3: 右轮编码器输入捕获
 * - TIM4: 车轮PWM控制
 * - TIM5: 方向舵控制
 * - TIM6: 系统时基
 * - TIM8: 高精度PWM输出
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */


#include "tim.h"

// 定时器句柄定义
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

/* TIM1 - 高精度系统同步定时器 */
void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance = TIM1;                    // 高级定时器
    htim1.Init.Prescaler = 0;                 // 预分频器
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 4800-1;              // 周期值
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_OC_Init(&htim1) != HAL_OK) {  // 输出比较
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) { // PWM输出
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    // CH1: 切换模式输出
    sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
    sConfigOC.Pulse = 500;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_1);

    // CH2: 激活模式输出
    sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
    if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_2);

    // CH3: 强制激活模式
    sConfigOC.OCMode = TIM_OCMODE_FORCED_ACTIVE;
    if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_3);

    // CH4: PWM1模式，用于LED控制
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }

    // 配置死区和刹车功能
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim1);
}

/* TIM2 - 左轮编码器输入捕获 */
void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;                   // 通用定时器
    htim2.Init.Prescaler = 144-1;           // 预分频: 72MHz/144 = 500kHz
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 10000-1;            // 周期: 20ms (50Hz)
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    // CH1, CH2: PWM1模式，用于编码器A/B相
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 750;                  // 1.5ms脉冲宽度
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim2);
}

/* TIM3 - 右轮编码器输入捕获 */
void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance = TIM3;                   // 通用定时器
    htim3.Init.Prescaler = 144-1;           // 预分频: 72MHz/144 = 500kHz
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 10000-1;            // 周期: 20ms (50Hz)
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    // CH1, CH2, CH3: PWM1模式，用于编码器和转向控制
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 750;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim3);
}

/* TIM4 - 车轮PWM控制 */
void MX_TIM4_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim4.Instance = TIM4;                   // 通用定时器
    htim4.Init.Prescaler = 0;               // 直接使用时钟
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 4800-1;             // PWM周期
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    // CH1, CH2: PWM1模式，用于左右轮电机控制
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;                    // 初始占空比为0
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim4);
}

/* TIM5 - 方向舵控制 */
void MX_TIM5_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim5.Instance = TIM5;                   // 通用定时器
    htim5.Init.Prescaler = 144-1;           // 预分频: 72MHz/144 = 500kHz
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 9999;               // 周期: 20ms (50Hz)
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    // CH3, CH4: PWM1模式，用于方向舵控制
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 750;                  // 1.5ms中位脉冲
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim5);
}

/* TIM6 - 系统时基定时器 */
void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim6.Instance = TIM6;                  // 基础定时器
    htim6.Init.Prescaler = 72-1;           // 预分频: 72MHz/72 = 1MHz
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 0xFFFF;            // 最大计数值
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    // 配置定时器中断
    HAL_NVIC_SetPriority(TIM6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_IRQn);
}

/* TIM8 - 高精度PWM输出 */
void MX_TIM8_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim8.Instance = TIM8;                  // 高级定时器
    htim8.Init.Prescaler = 0;              // 直接使用时钟
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = 4800-1;            // PWM周期
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim8) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    // CH1: PWM1模式
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;                   // 初始占空比为0
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    // 配置死区和刹车功能
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim8);
}

// 定时器时钟使能
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
    if(tim_baseHandle->Instance==TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
    }
    else if(tim_baseHandle->Instance==TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
    }
    else if(tim_baseHandle->Instance==TIM3) {
        __HAL_RCC_TIM3_CLK_ENABLE();
    }
    else if(tim_baseHandle->Instance==TIM4) {
        __HAL_RCC_TIM4_CLK_ENABLE();
    }
    else if(tim_baseHandle->Instance==TIM5) {
        __HAL_RCC_TIM5_CLK_ENABLE();
    }
    else if(tim_baseHandle->Instance==TIM6) {
        __HAL_RCC_TIM6_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM6_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM6_IRQn);
    }
    else if(tim_baseHandle->Instance==TIM8) {
        __HAL_RCC_TIM8_CLK_ENABLE();
    }
}

// 定时器GPIO后初始化
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(timHandle->Instance==TIM1) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        // TIM1_CH1N, TIM1_CH2N, TIM1_CH3N -> PB13, PB14, PB15
        GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        // TIM1_CH4 -> PA11 (LED_1)
        GPIO_InitStruct.Pin = LED_1_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(LED_1_GPIO_Port, &GPIO_InitStruct);
    }
    else if(timHandle->Instance==TIM2) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        // TIM2_CH1, TIM2_CH2 -> PA0, PA1 (编码器1 A/B相)
        GPIO_InitStruct.Pin = ENC_CH1_A_Pin|ENC_CH1_B_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if(timHandle->Instance==TIM3) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        // TIM3_CH1, TIM3_CH2 -> PA6, PA7 (编码器2 A/B相)
        GPIO_InitStruct.Pin = ENC_CH2_A_Pin|ENC_CH2_B_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // TIM3_CH3 -> PB0 (转向B0)
        GPIO_InitStruct.Pin = Steer_B0_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(Steer_B0_GPIO_Port, &GPIO_InitStruct);
    }
    else if(timHandle->Instance==TIM4) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        // TIM4_CH1 -> PB6 (右轮PWM)
        GPIO_InitStruct.Pin = Wheel_Right_PWM_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(Wheel_Right_PWM_GPIO_Port, &GPIO_InitStruct);

        // TIM4_CH2 -> PB7 (左轮PWM)
        GPIO_InitStruct.Pin = Wheel_Left_PWM_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(Wheel_Left_PWM_GPIO_Port, &GPIO_InitStruct);
    }
    else if(timHandle->Instance==TIM5) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        // TIM5_CH3, TIM5_CH4 -> PA2, PA3 (转向A2, A3)
        GPIO_InitStruct.Pin = Steer_A2_Pin|Steer_A3_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if(timHandle->Instance==TIM8) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
        // TIM8_CH1 -> PC6
        GPIO_InitStruct.Pin = GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    }
}

// 定时器反初始化
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{
    if(tim_baseHandle->Instance==TIM1) {
        __HAL_RCC_TIM1_CLK_DISABLE();
    }
    else if(tim_baseHandle->Instance==TIM2) {
        __HAL_RCC_TIM2_CLK_DISABLE();
    }
    else if(tim_baseHandle->Instance==TIM3) {
        __HAL_RCC_TIM3_CLK_DISABLE();
    }
    else if(tim_baseHandle->Instance==TIM4) {
        __HAL_RCC_TIM4_CLK_DISABLE();
    }
    else if(tim_baseHandle->Instance==TIM5) {
        __HAL_RCC_TIM5_CLK_DISABLE();
    }
    else if(tim_baseHandle->Instance==TIM6) {
        __HAL_RCC_TIM6_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(TIM6_IRQn);
    }
    else if(tim_baseHandle->Instance==TIM8) {
        __HAL_RCC_TIM8_CLK_DISABLE();
    }
}

/*
 * 定时器配置总结：
 * 
 * TIM1 (高级定时器): 
 * - 用途: LED控制、系统同步
 * - 配置: 4800周期，多种输出模式
 * - 输出: CH1/CH2/CH3为互补输出，CH4为LED PWM
 * 
 * TIM2 (通用定时器):
 * - 用途: 左轮编码器输入捕获
 * - 配置: 500kHz计数频率，20ms周期
 * - 连接: 编码器1的A/B相 (PA0, PA1)
 * 
 * TIM3 (通用定时器):
 * - 用途: 右轮编码器输入捕获
 * - 配置: 500kHz计数频率，20ms周期
 * - 连接: 编码器2的A/B相 (PA6, PA7)，转向控制 (PB0)
 * 
 * TIM4 (通用定时器):
 * - 用途: 车轮PWM控制
 * - 配置: 4800周期PWM
 * - 输出: 左右轮电机控制 (PB6, PB7)
 * 
 * TIM5 (通用定时器):
 * - 用途: 方向舵控制
 * - 配置: 50Hz PWM (20ms周期)
 * - 输出: 舵机控制信号 (PA2, PA3)
 * 
 * TIM6 (基础定时器):
 * - 用途: 系统时基
 * - 配置: 中断驱动，1MHz计数频率
 * 
 * TIM8 (高级定时器):
 * - 用途: 高精度PWM输出
 * - 配置: 4800周期，支持死区和刹车功能
 * - 输出: 高精度控制信号 (PC6)
 * 
 * 在智能小车系统中的作用：
 * 1. 精确的电机控制 - 通过PWM信号控制车轮速度
 * 2. 编码器反馈 - 通过输入捕获获取轮子转速
 * 3. 舵机控制 - 实现方向控制
 * 4. 系统同步 - 多定时器协调工作
 * 5. LED指示 - 系统状态显示
 */




