/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

// GPIO初始化函数
void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

/*
 * 这个头文件的功能说明：
 * 
 * 1. 头文件保护：使用#ifndef宏防止重复包含
 * 2. 依赖包含：包含main.h主头文件
 * 3. GPIO初始化：提供MX_GPIO_Init()函数用于初始化所有GPIO引脚
 * 
 * 在完整的GPIO.c文件中，MX_GPIO_Init()函数会：
 * - 配置所有GPIO引脚的工作模式（输入/输出/复用功能）
 * - 设置引脚速度、上拉/下拉电阻等参数
 * - 将引脚配置为特定功能（如LED、按键、外设信号线等）
 * 
 * 从前面的main.h文件看，这个项目包含：
 * - LED指示灯（LED_1到LED_4）
 * - 开关输入（Switch_1到Switch_4）
 * - 编码器接口（ENC_CH1_A, ENC_CH1_B等）
 * - PWM输出（Wheel_Left_PWM, Wheel_Right_PWM等）
 * - 方向控制（Wheel_Left_IO, Wheel_Right_IO等）
 * - 转向控制（Steer_A2, Steer_A3等）
 * - PPM输入（遥控信号）
 * 
 * GPIO初始化是系统启动时的重要步骤
 */




