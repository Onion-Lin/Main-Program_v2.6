/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c  YSL
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023-2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * * * * * * * * * * * *
 *....................................................................................................*
 *....................................................................................................*
 *....................................................................................................*
 *.............................................tttttttttttttttttttttt.................................*
 *............................................ttttttttttttttttttttttt.................................*
 *............................................tttttttttttttttttttttt..................................*
 *...........................................tttttttttttttttttttttt...................................*
 *..........................................ttttttttttttttttttttttt...................................*
 *..........................................tttttttttttttttttttttt....................................*
 *..........................................ttttttttttttttttttttt.....................................*
 *....................................................................................................*
 *....................ttttttttttttt.......tttttttttttttttttttttt...tttttttttt.........................*
 *...................tttttttttttt.....ttttttttttttttttttttttttt....tttttttttt.........................*
 *...................tttttttt......tttttttttttttttttttttttttttt...tttttttttt..........................*
 *..................ttttt......ttttttttttttttttttttttttttttttt...tttttttttt...........................*
 *.................ttt......ttttttttttttttttttttttttttttttttt....tttttttttt...........................*
 *................tt....ttttttttttttttttttttttttttttttttttttt...tttttttttt............................*
 *...................ttttttttttttttttttttttttttttttttttttttt....ttttttttt.............................*
 *................................ttttttttttttttttttttttttt...........................................*
 *...............................tttttttttttttttttttttttttt...........................................*
 *..............................tttttttttttttttttttttttttt............................................*
 *..............................ttttttttttttttttttttttttt.............................................*
 *.............................tttttttttttttttttttttttttt.............................................*
 *............................ttttttttttttttttttttttttttt........ttt..................................*
 *............................ttttttttttttttttttttttttttttttttttttt...................................*
 *...........................ttttttttttttttttttttttttttttttttttttt....................................*
 *...........................tttttttttttttttttttttttttttttttttttt.....................................*
 *...........................tttttttttttttttttttttttttttttttttttt.....................................*
 *...........................ttttttttttttttttttttttttttttttttttt......................................*
 *............................ttttttttttttttttttttttttttttttttt.......................................*
 *.............................ttttttttttttttttttttttttttttttt........................................*
 *..............................tttttttttttttttttttttttttttt..........................................*
 *.................................ttttttttttttttttttttt..............................................*
 *........................................tttttt......................................................*
 *....................................................................................................*
 *....................................................................................................*
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * * * * * * * * * * * *
 */

/* USER CODE END Header */
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "headfiles.h"
#include "tim.h"
#include "usart.h"

/* 舵机PWM参数定义 */
#define Steer_PWM_Center 792                          // 舵机中值   调低偏右
#define Steer_PWM_Limit_Lift (Steer_PWM_Center - 85)  // 舵机左转限幅
#define Steer_PWM_Limit_Right (Steer_PWM_Center + 85) // 舵机右转限幅

/* 全局变量定义 */
int16_t Dir_Output;             // 方向输出值
uint16_t SteerPWM = Steer_PWM_Center; // 舵机PWM输出值
void SystemClock_Config(void);

// 拨码开关状态缓存
static uint8_t last_switch_count = 0xFF;

// 蓝牙遥控变量
volatile uint8_t bt_rx_buf = 0;     // 蓝牙接收缓冲区
volatile uint8_t bt_cmd_ready = 0;  // 新命令标志

// 左电机驱动变量
int16_t Wheel_Left_Speed;      // 驱动电机速度期望
int16_t Std_Speed = 650;       // 基准速度
uint16_t Wheel_Left_PWM = 0;   // 驱动电机PWM输出值
uint8_t Wheel_Left_IO = RESET; // 驱动电机IO口电平

// 右电机驱动变量
int16_t Wheel_Right_Speed;     // 驱动电机速度期望
uint16_t Wheel_Right_PWM = 0;  // 驱动电机PWM输出值
uint8_t Wheel_Right_IO = RESET;// 驱动电机IO口电平

/**
 * @brief  主函数
 * @retval int
 */

int main(void) {
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();

  PWM_Init();
  RC_Init();
  ADC_Init();

  // 启动蓝牙串口中断接收（USART3）
  HAL_UART_Receive_IT(&huart_USER, (uint8_t *)&bt_rx_buf, 1);

  // 初始状态：停车
  Wheel_Left_Speed = 0;
  Wheel_Right_Speed = 0;
  Dir_Output = Steer_PWM_Center;

  while (1) {
    // 1. 读取拨码开关设定速度
    Switch();

    // 2. 处理蓝牙命令
    if (bt_cmd_ready) {
      bt_cmd_ready = 0;
      switch (bt_rx_buf) {
        case 'W': case 'w': // 前进
          Wheel_Left_Speed = Std_Speed;
          Wheel_Right_Speed = Std_Speed;
          Dir_Output = Steer_PWM_Center;
          break;
        case 'S': case 's': // 停车
          Wheel_Left_Speed = 0;
          Wheel_Right_Speed = 0;
          Dir_Output = Steer_PWM_Center;
          break;
        case 'A': case 'a': // 左转前进
          Wheel_Left_Speed = Std_Speed * 0.4;
          Wheel_Right_Speed = Std_Speed;
          Dir_Output = Steer_PWM_Center - 60;
          break;
        case 'D': case 'd': // 右转前进
          Wheel_Left_Speed = Std_Speed;
          Wheel_Right_Speed = Std_Speed * 0.4;
          Dir_Output = Steer_PWM_Center + 60;
          break;
        case 'X': case 'x': // 后退
          Wheel_Left_Speed = -Std_Speed;
          Wheel_Right_Speed = -Std_Speed;
          Dir_Output = Steer_PWM_Center;
          break;
        case '+': // 加速
          Std_Speed += 50;
          if (Std_Speed > 4000) Std_Speed = 4000;
          Usart_SendString(&huart_USER, (uint8_t *)"SPD+ ");
          break;
        case '-': // 减速
          Std_Speed -= 50;
          if (Std_Speed < 100) Std_Speed = 100;
          Usart_SendString(&huart_USER, (uint8_t *)"SPD- ");
          break;
      }
      // 重新启动接收
      HAL_UART_Receive_IT(&huart_USER, (uint8_t *)&bt_rx_buf, 1);
    }

    // 3. 使能开关安全控制
    if (Switch_EN_ON) {
      HAL_GPIO_WritePin(Enable_IO_GPIO_Port, Enable_IO_Pin, GPIO_PIN_SET);
    } else {
      Wheel_Left_Speed = 0;
      Wheel_Right_Speed = 0;
      Dir_Output = Steer_PWM_Center;
      HAL_GPIO_WritePin(Enable_IO_GPIO_Port, Enable_IO_Pin, GPIO_PIN_RESET);
    }

    // 4. 速度限幅
    Wheel_Left_Speed = Data_Limit(Wheel_Left_Speed, -4000, 4000);
    Wheel_Right_Speed = Data_Limit(Wheel_Right_Speed, -4000, 4000);

    // 5. 舵机限幅
    Dir_Output = Data_Limit(Dir_Output, Steer_PWM_Limit_Lift, Steer_PWM_Limit_Right);
    SteerPWM = (uint16_t)Dir_Output;

    // 6. 左电机PWM和方向
    if (Wheel_Left_Speed >= 0) {
      Wheel_Left_PWM = Wheel_Left_Speed;
      Wheel_Left_IO = GPIO_PIN_RESET;
    } else {
      Wheel_Left_PWM = 4800 + Wheel_Left_Speed;
      Wheel_Left_IO = GPIO_PIN_SET;
    }

    // 7. 右电机PWM和方向
    if (Wheel_Right_Speed >= 0) {
      Wheel_Right_PWM = Wheel_Right_Speed;
      Wheel_Right_IO = GPIO_PIN_RESET;
    } else {
      Wheel_Right_PWM = 4800 + Wheel_Right_Speed;
      Wheel_Right_IO = GPIO_PIN_SET;
    }

    // 8. PWM输出
    PWM_SetDuty(PWM_TIM3_CH3_B0, SteerPWM);        // 舵机
    PWM_SetDuty(PWM_TIM4_CH2_B7, Wheel_Left_PWM);  // 左轮
    PWM_SetDuty(PWM_TIM4_CH1_B6, Wheel_Right_PWM); // 右轮
    HAL_GPIO_WritePin(Wheel_Left_IO_GPIO_Port, Wheel_Left_IO_Pin, Wheel_Left_IO);
    HAL_GPIO_WritePin(Wheel_Right_IO_GPIO_Port, Wheel_Right_IO_Pin, Wheel_Right_IO);

    // 9. 串口调试
    VOFA_JustFloat(&huart_DBG);

    HAL_Delay(10);
  }
}

/**
 * @brief 蓝牙串口接收完成回调
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART3) {
    bt_cmd_ready = 1;
  }
}

/* 拨码开关 — 设定蓝牙遥控基准速度 */
void Switch(void) {
  Switch_GetCode();
  uint8_t bit1 = Switch_GetState_Index(Switch_Index_1);
  uint8_t bit2 = Switch_GetState_Index(Switch_Index_2);
  uint8_t bit3 = Switch_GetState_Index(Switch_Index_3);
  uint8_t bit4 = Switch_GetState_Index(Switch_Index_4);
  uint8_t switch_code = (bit4 << 3) | (bit3 << 2) | (bit2 << 1) | (bit1 << 0);

 if (switch_code != last_switch_count) {
    last_switch_count = switch_code;
    switch(switch_code) {
    case 0b0000: Std_Speed = 550;  break;
    case 0b0001: Std_Speed = 650;  break;
    case 0b0010: Std_Speed = 650;  break;
    case 0b0100: Std_Speed = 800;  break;
    case 0b1000: Std_Speed = 1000; break;
    default:     Std_Speed = 650;  break;
    }
    PWM_SetDuty(PWM_TIM1_CH4_A11,Switch_GetState_Index(Switch_Index_1) * PWM_BreathDuty());
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, Switch_GetState_Index(Switch_Index_2));
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, Switch_GetState_Index(Switch_Index_3));
    HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, Switch_GetState_Index(Switch_Index_4));
  }
}



/**
 * @brief 系统时钟配置 - 72MHz
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  // 配置HSE和PLL
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9; // 8MHz * 9 = 72MHz
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  // 配置系统时钟
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; // 36MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; // 72MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  // ADC时钟配置
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6; // 6MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}

/**
 * @brief 定时器中断回调函数
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM7) {
    HAL_IncTick(); // 增加系统滴答计数
  }
}

/**
 * @brief 错误处理函数
 */
void Error_Handler(void) {
  __disable_irq();
  while (1) {
    // 错误发生时的处理
  }
}



 




