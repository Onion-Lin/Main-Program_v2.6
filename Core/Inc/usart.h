/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  			这段代码是STM32F1微控制器的USART（通用同步/异步收发器）驱动头文件。
  			
			USART1 (huart1) - 用作调试端口，用于输出调试信息和系统状态
			USART3 (huart3) - 用作用户端口，用于与外部设备或上位机通信
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

// UART句柄声明
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

// USART初始化函数
void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */

// 用户自定义串口发送函数
void Usart_SendByte(UART_HandleTypeDef *huart, uint8_t str);        // 发送单字节
void Usart_SendString(UART_HandleTypeDef *huart, uint8_t *str);     // 发送字符串
void Usart_SendArr(UART_HandleTypeDef *huart, uint8_t *arr, uint16_t size);  // 发送数组

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

/*
 * 这个头文件的功能说明：
 * 
 * 1. 头文件保护：使用#ifndef宏防止重复包含
 * 2. 依赖包含：包含main.h主头文件
 * 3. UART句柄：声明两个UART句柄（huart1和huart3）
 * 4. 初始化函数：为两个UART提供初始化函数
 * 5. 自定义发送函数：提供发送字节、字符串、数组的便捷函数
 * 
 * 从main.h的宏定义看：
 * - #define huart_DBG huart1 -> huart1用于调试信息输出
 * - #define huart_USER huart3 -> huart3用于用户数据通信
 * 
 * 应用场景：
 * - USART1 (huart1/debug)：用于调试信息输出、系统状态监控
 * - USART3 (huart3/user)：用于用户数据交换、上位机通信
 * 
 * 自定义发送函数提供了更便捷的串口数据发送接口：
 * - Usart_SendByte: 发送单个字节
 * - Usart_SendString: 发送字符串（以'\0'结尾）
 * - Usart_SendArr: 发送指定长度的数据数组
 */




