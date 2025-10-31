/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
			1. **PWM生成**：控制电机速度和方向
			2. **编码器接口**：读取电机转速和位置
			3. **ADC触发**：定时启动模数转换
			4. **系统时基**：提供精确的时间基准
			5. **信号解码**：处理PPM遥控信号
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

// 定时器句柄声明
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

// 定时器初始化函数
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM5_Init(void);
void MX_TIM6_Init(void);
void MX_TIM8_Init(void);

// MspPostInit函数（用于PWM输出等需要额外GPIO配置的功能）
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

/*
 * 这个头文件的功能说明：
 * 
 * 1. 头文件保护：使用#ifndef宏防止重复包含
 * 2. 依赖包含：包含main.h主头文件
 * 3. 定时器句柄：声明7个定时器的句柄（TIM1-TIM8，但不包括TIM7）
 * 4. 初始化函数：为每个定时器提供初始化函数
 * 5. MspPostInit：用于需要额外GPIO配置的定时器功能（如PWM输出）
 * 
 * 定时器功能分析：
 * - TIM1: 高级定时器，可能用于复杂的PWM控制
 * - TIM2-TIM5: 通用定时器，可能用于编码器输入、定时采样等
 * - TIM6: 基本定时器，可能用于系统时基或ADC触发
 * - TIM8: 高级定时器，可能用于电机控制
 * 
 * 结合前面的GPIO配置，可能的应用包括：
 * - 使用定时器生成PWM信号控制电机（Wheel_Left_PWM, Wheel_Right_PWM）
 * - 使用定时器捕获编码器信号（ENC_CH1_A/B, ENC_CH2_A/B）
 * - 使用定时器进行周期性ADC采样
 * - 使用定时器产生PPM解码时基
 */




