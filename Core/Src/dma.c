/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of all the requested memory to memory DMA transfers.
 * DMA (Direct Memory Access) 配置文件
 * 
 * 功能概述：
 * - 启用DMA1控制器时钟
 * - 配置DMA中断优先级
 * - 启用特定DMA通道中断
 * 
 * 硬件特性：
 * - STM32F1系列有DMA1和DMA2两个DMA控制器
 * - DMA1有7个通道，DMA2有5个通道
 * - 每个通道可连接不同的外设请求
 */
 
/* USER CODE END Header */



#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @brief DMA初始化函数
  * @note  启用DMA1控制器并配置中断
  * @retval None
  */
void MX_DMA_Init(void)
{
    /* 使能DMA1控制器时钟 */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA中断初始化 */
    /* DMA1_Channel1_IRQn 中断配置 - 通常用于ADC */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);  // 设置中断优先级
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);          // 使能中断线

    /* DMA1_Channel4_IRQn 中断配置 - 通常用于USART1_TX */
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);  // 设置中断优先级
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);          // 使能中断线
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/*
 * DMA配置分析：
 * 
 * 1. DMA1 Channel 1:
 *    - 在这个智能小车系统中，可能用于ADC采样数据的自动传输
 *    - 避免CPU频繁读取ADC数据，提高系统效率
 * 
 * 2. DMA1 Channel 4:
 *    - 通常连接到USART1_TX
 *    - 用于串口数据的后台传输
 *    - 提高串口通信效率，释放CPU资源
 * 
 * DMA工作原理：
 * - 在设定条件下自动在存储器和外设之间传输数据
 * - 不需要CPU干预，提高系统性能
 * - 支持多种传输模式（单次、循环、乒乓等）
 * 
 * 中断配置：
 * - 优先级设置为0，表示最高优先级
 * - 当DMA传输完成或出错时触发中断
 * - 在中断服务程序中处理后续操作
 * 
 * 在智能小车系统中的应用：
 * - ADC采样：自动将传感器数据存入内存
 * - 串口通信：自动发送调试信息或接收控制指令
 * - 减少CPU负担，提高实时性
 */

/*
 * 典型的DMA中断服务程序（在其他文件中实现）：
 */




