/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
 * GPIO (General Purpose Input/Output) 配置文件
 * 
 * 功能概述：
 * - 配置小车的控制接口
 * - 设置输入输出引脚
 * - 实现外部中断功能
 * 
 * 硬件特性：
 * - 支持多种IO模式（输入、输出、复用、模拟）
 * - 内置上拉/下拉电阻
 * - 外部中断功能
  ******************************************************************************
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


#include "gpio.h"
#include "RC.h"

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/

/** 配置引脚为以下模式:
        * 模拟输入
        * 数字输入
        * 数字输出
        * 事件输出
        * 外部中断
*/
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能GPIO端口时钟 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 配置输出引脚初始电平 */
    HAL_GPIO_WritePin(GPIOC, Enable_IO_Pin|LED_3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_4_Pin|Wheel_Left_IO_Pin|Wheel_Right_IO_Pin, GPIO_PIN_RESET);

    /* 配置输出引脚 - 使能控制和LED_3 */
    GPIO_InitStruct.Pin = Enable_IO_Pin|LED_3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;          // 推挽输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;                  // 无上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;       // 高速
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* 配置外部中断引脚 - PPM接收 */
    GPIO_InitStruct.Pin = PPM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;        // 下降沿中断
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PPM_GPIO_Port, &GPIO_InitStruct);

    /* 配置输入引脚 - 按键开关 */
    GPIO_InitStruct.Pin = Switch_4_Pin|Switch_2_Pin|Switch_3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;             // 输入模式
    GPIO_InitStruct.Pull = GPIO_PULLUP;                 // 上拉电阻
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* 配置输入引脚 - 开关1 */
    GPIO_InitStruct.Pin = Switch_1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(Switch_1_GPIO_Port, &GPIO_InitStruct);

    /* 配置输入引脚 - EN开关 */
    GPIO_InitStruct.Pin = Switch_EN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;                 // 无上下拉
    HAL_GPIO_Init(Switch_EN_GPIO_Port, &GPIO_InitStruct);
		
		/*配置输入引脚 - 干簧管*/
		GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;                 // 上拉
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* 配置输出引脚 - LED_2 */
    GPIO_InitStruct.Pin = LED_2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LED_2_GPIO_Port, &GPIO_InitStruct);

    /* 配置输出引脚 - LED_4和车轮控制 */
    GPIO_InitStruct.Pin = LED_4_Pin|Wheel_Left_IO_Pin|Wheel_Right_IO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 初始化外部中断 */
    HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);             // 设置中断优先级
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);                     // 使能中断
}

/* 用户自定义代码 */
/**
  * @brief  外部中断回调函数
  * @param  GPIO_Pin: 指定连接到外部中断线的引脚
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_PIN_3 == GPIO_Pin)
    {
        RC_ReceiveCallback();  // 调用遥控接收回调函数
    }
}

/*
 * GPIO配置分析：
 * 
 * 1. 输出引脚 (推挽输出, 高速):
 *    - Enable_IO_Pin: 小车使能控制
 *    - LED_2_Pin, LED_3_Pin, LED_4_Pin: 状态指示灯
 *    - Wheel_Left_IO_Pin, Wheel_Right_IO_Pin: 车轮控制信号
 * 
 * 2. 输入引脚 (带内部上拉):
 *    - Switch_1_Pin, Switch_2_Pin, Switch_3_Pin, Switch_4_Pin: 用户按键
 *    - Switch_EN_Pin: 使能开关
 * 
 * 3. 外部中断引脚:
 *    - PPM_Pin: PPM信号输入，用于遥控接收
 *    - 触发方式: 下降沿中断
 * 
 * 在智能小车系统中的应用：
 * - 按键控制: 用户模式选择和参数设置
 * - LED指示: 系统状态显示
 * - 车轮控制: 左右轮使能信号
 * - 遥控接口: PPM信号接收和处理
 * - 安全使能: 系统安全控制
 */

/*
 * 引脚功能分配：
 * 
 * 端口C:
 * - Enable_IO_Pin: 小车使能控制
 * - LED_3_Pin: 状态LED 3
 * - Switch_4_Pin, Switch_2_Pin, Switch_3_Pin: 用户按键
 * 
 * 端口B:
 * - LED_4_Pin: 状态LED 4
 * - Wheel_Left_IO_Pin: 左轮使能
 * - Wheel_Right_IO_Pin: 右轮使能
 * 
 * 端口A:
 * - PPM_Pin (PA3): PPM遥控信号输入
 * - Switch_1_Pin (PA0): 用户按键1
 * 
 * 端口D:
 * - Switch_EN_Pin: 使能开关
 * 
 * LED_2_Pin: 状态LED 2
 * 
 * 外部中断机制:
 * - PPM信号下降沿触发
 * - 调用RC_ReceiveCallback()处理遥控指令
 * - 实现非阻塞的遥控信号接收
 */

/*
 * 安全特性:
 * - 所有控制输出初始化为低电平(安全状态)
 * - 使能开关确保系统安全启动
 * - 硬件级的紧急停止功能
 */




