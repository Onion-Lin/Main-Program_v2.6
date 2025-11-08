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

/* 转弯分段参数 */
#define TURN_THRESHOLD_HIGH 25    // 大转弯阈值
#define TURN_THRESHOLD_LOW 10     // 小转弯阈值
#define STRAIGHT_THRESHOLD 10      // 直行阈值

/* 电感检测阈值 */
#define AD_THRESHOLD 150           // 电感检测阈值

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
// 干簧管滑动窗口滤波相关
#define Stop_Fliter_SIZE 5
static GPIO_PinState reed_window[Stop_Fliter_SIZE] = {GPIO_PIN_RESET};
static int reed_window_index = 0;

// 电感滑动窗口相关（利用现有数组）
#define ADC_Fliter_SIZE 3
static int16_t Fliter_Left_Window[ADC_Fliter_SIZE] = {0, 0, 0};
static int16_t Fliter_Right_Window[ADC_Fliter_SIZE] = {0, 0, 0};
static int Pins_window_index = 0;

/**
 * @brief 电感值滑动窗口滤波
 * @param raw_value 原始ADC值
 * @param window 滑动窗口数组
 * @param window_size 窗口大小
 * @param index 窗口索引
 * @return 滤波后的电感值
 */
int16_t ADC_Filter(int16_t raw_value, int16_t *window, int window_size, int *index);

/**
 * @brief 干簧管状态滑动窗口滤波
 * @param current_state 当前干簧管状态
 * @return 滤波后的干簧管状态
 */
GPIO_PinState Stop_Filter(GPIO_PinState current_state);

/**
 * @brief 改进的微分项计算，带滤波功能
 * @param current_error 当前误差
 * @param dt 时间间隔
 * @return 滤波后的微分值
 */
float Kd_Math(float current_error, float dt);

/**
 * @brief 改进的方向环PD控制算法
 * @param error 方向误差
 * @param is_turning 是否处于转弯状态
 * @return 控制输出
 */
float PD_Control(float error, uint8_t is_turning);

/**
 * @brief 改进的差速PD控制
 * @param steer_output 舵机输出值
 * @param error 方向误差
 * @param is_turning 是否处于转弯状态
 * @return 差速输出
 */
float Wheel_PD_Control(float steer_output, float error, uint8_t is_turning);

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




