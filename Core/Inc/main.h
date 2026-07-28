/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* 转弯分段参数（保留供参考） */
#define TURN_THRESHOLD_HIGH 25
#define TURN_THRESHOLD_LOW 10

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
// 串口定义
#define huart_DBG huart1
#define huart_USER huart3

// 定时器定义
#define htim_RC htim6

// GPIO引脚定义 - 使能控制
#define Enable_IO_Pin GPIO_PIN_13
#define Enable_IO_GPIO_Port GPIOC

// PPM输入
#define PPM_Pin GPIO_PIN_3
#define PPM_GPIO_Port GPIOC
#define PPM_EXTI_IRQn EXTI3_IRQn

// 编码器1 (通道A和B)
#define ENC_CH1_A_Pin GPIO_PIN_0
#define ENC_CH1_A_GPIO_Port GPIOA
#define ENC_CH1_B_Pin GPIO_PIN_1
#define ENC_CH1_B_GPIO_Port GPIOA

// 转向控制引脚
#define Steer_A2_Pin GPIO_PIN_2
#define Steer_A2_GPIO_Port GPIOA
#define Steer_A3_Pin GPIO_PIN_3
#define Steer_A3_GPIO_Port GPIOA

// 编码器2 (通道A和B)
#define ENC_CH2_A_Pin GPIO_PIN_6
#define ENC_CH2_A_GPIO_Port GPIOA
#define ENC_CH2_B_Pin GPIO_PIN_7
#define ENC_CH2_B_GPIO_Port GPIOA

// 转向控制引脚
#define Steer_B0_Pin GPIO_PIN_0
#define Steer_B0_GPIO_Port GPIOB

// 开关引脚
#define Switch_4_Pin GPIO_PIN_7
#define Switch_4_GPIO_Port GPIOC
#define Switch_2_Pin GPIO_PIN_8
#define Switch_2_GPIO_Port GPIOC
#define Switch_3_Pin GPIO_PIN_9
#define Switch_3_GPIO_Port GPIOC
#define Switch_1_Pin GPIO_PIN_8
#define Switch_1_GPIO_Port GPIOA

// 开关使能
#define Switch_EN_Pin GPIO_PIN_12
#define Switch_EN_GPIO_Port GPIOA

// LED指示灯
#define LED_1_Pin GPIO_PIN_11
#define LED_1_GPIO_Port GPIOA
#define LED_2_Pin GPIO_PIN_15
#define LED_2_GPIO_Port GPIOA
#define LED_3_Pin GPIO_PIN_12
#define LED_3_GPIO_Port GPIOC
#define LED_4_Pin GPIO_PIN_3
#define LED_4_GPIO_Port GPIOB

// 轮子PWM控制
#define Wheel_Right_PWM_Pin GPIO_PIN_6
#define Wheel_Right_PWM_GPIO_Port GPIOB
#define Wheel_Left_PWM_Pin GPIO_PIN_7
#define Wheel_Left_PWM_GPIO_Port GPIOB

// 轮子方向控制
#define Wheel_Left_IO_Pin GPIO_PIN_8
#define Wheel_Left_IO_GPIO_Port GPIOB
#define Wheel_Right_IO_Pin GPIO_PIN_9
#define Wheel_Right_IO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
//拨码开关控制函数
void Switch(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/* 
 * 这个头文件的作用：
 * 1. 防止重复包含：使用#ifndef宏保护
 * 2. 兼容C++：extern "C" {}块
 * 3. 包含HAL库：STM32F1硬件抽象层
 * 4. 引脚映射：将物理引脚映射为有意义的名称
 * 5. 外设映射：串口、定时器等外设实例
 * 
 * 从引脚定义可以看出这是一个机器人或小车控制系统，包含：
 * - 双编码器（ENC_CH1, ENC_CH2）用于位置反馈
 * - 四个开关用于用户输入
 * - 四个LED用于状态指示
 * - 左右轮PWM和方向控制
 * - 转向控制引脚
 * - PPM输入（可能用于遥控器信号）
 */




