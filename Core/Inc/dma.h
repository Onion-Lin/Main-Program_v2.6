/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.h
  * @brief   This file contains all the function prototypes for
  *          the dma.c file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DMA_H__
#define __DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* DMA memory to memory transfer handles -------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

// 主要函数：初始化DMA
void MX_DMA_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __DMA_H__ */

/*
 * 这个头文件的功能说明：
 * 
 * 1. 头文件保护：使用#ifndef宏防止重复包含
 * 2. 依赖包含：包含main.h主头文件
 * 3. DMA初始化：提供MX_DMA_Init()函数用于初始化DMA
 * 
 * DMA（Direct Memory Access）功能说明：
 * - 允许外设与内存之间直接传输数据，无需CPU干预
 * - 提高数据传输效率，降低CPU负载
 * - 常用于ADC采样、UART通信、存储器操作等场景
 * 
 * 从代码看，这个DMA配置可能用于：
 * - ADC数据采集（配合前面的ADC驱动）
 * - UART数据传输
 * - 定时器相关操作
 * 
 * 注释中的"memory to memory transfer handles"表明
 * 可能支持内存到内存的数据传输功能
 */




