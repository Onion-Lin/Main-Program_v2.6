/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
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
#define Steer_PWM_Limit_Lift (Steer_PWM_Center - 70)  // 舵机左转限幅
#define Steer_PWM_Limit_Right (Steer_PWM_Center + 70) // 舵机右转限幅

/* 全局变量定义 */
int16_t AD_Left, AD_Right;            // 左右电感的ADC值
float Dir_Err;                        // 方向偏差
int16_t Dir_Output;                   // 方向输出值
uint16_t SteerPWM = Steer_PWM_Center; // 舵机PWM输出值
void SystemClock_Config(void);

// 拨码开关状态缓存
static uint8_t last_switch_count = 0xFF; // 初始化为无效状态
static uint8_t current_switch_count = 0;

// PD与差速变量
float Dir_Err_Wheel = 0;
float Dir_Err_Wheel_Limit_Low = -400; // 差速限幅
float Dir_Err_Wheel_Limit_High = 400;
float Kp, Kp_s, Kp_t_, Kd, Kd_s, Kd_t_; // 增加直行和转弯的微分参数
int16_t fliter_Buf_L[3] = {0, 0, 0};    // 原有数组，现在用作左电感滑动窗口
int16_t fliter_Buf_R[3] = {0, 0, 0};    // 原有数组，现在用作右电感滑动窗口
int16_t Kd_Change;
float Err_Tmp[3] = {0, 0, 0}; // 增加历史误差缓冲区
double fun;
int stop = 0;

// 干簧管停车相关变量
int reed_count = 0; // 干簧管计数器
int middle = 0;     // 直行转弯标志

// 左电机驱动变量
int16_t Wheel_Left_Speed;      // 驱动电机速度期望
int16_t Std_Speed;             // 基准速度
uint16_t Wheel_Left_PWM = 0;   // 驱动电机PWM输出值
uint8_t Wheel_Left_IO = RESET; // 驱动电机IO口电平

// 右电机驱动变量
int16_t Wheel_Right_Speed;      // 驱动电机速度期望
uint16_t Wheel_Right_PWM = 0;   // 驱动电机PWM输出值
uint8_t Wheel_Right_IO = RESET; // 驱动电机IO口电平

// PD控制相关
static float last_error = 0;          // 上一次误差
static float last_derivative = 0;     // 上一次微分值
static float filtered_derivative = 0; // 滤波后的微分值

/**
 * @brief  主函数
 * @retval int
 */

int main(void) {
  HAL_Init(); // 初始化HAL库
  SystemClock_Config();

  // 初始化所有外设
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

  // 用户初始化
  PWM_Init(); // PWM初始化
  RC_Init();  // 遥控接收初始化
  ADC_Init(); // ADC初始化

  // 初始化PD控制参数
  last_error = 0;
  last_derivative = 0;
  filtered_derivative = 0;

  // 初始化干簧管滑动窗口
  for (int i = 0; i < REED_WINDOW_SIZE; i++) {
    reed_window[i] = GPIO_PIN_RESET;
  }
  reed_window_index = 0;

  // 初始化电感滑动窗口
  for (int i = 0; i < INDUCTOR_WINDOW_SIZE; i++) {
    inductor_left_window[i] = 0;
    inductor_right_window[i] = 0;
  }
  inductor_window_index = 0;

  while (1) {
    // 1. 读取左右电感值 - 使用滑动窗口滤波
    int16_t raw_ad_left = ADC_GetValue_Channel(ADC1_CH04_PA4); // A4:R72-L10
    int16_t raw_ad_right =
        ADC_GetValue_Channel(ADC1_CH15_PC5); // C5:R1200-L1300

    // 使用滑动窗口滤波电感值
    AD_Left = InductorFilter(raw_ad_left, inductor_left_window,
                             INDUCTOR_WINDOW_SIZE, &inductor_window_index);
    AD_Right = InductorFilter(raw_ad_right, inductor_right_window,
                              INDUCTOR_WINDOW_SIZE, &inductor_window_index);

    AD_Left *= fun;
    AD_Right *= fun;

    // 2.计算输出，PID
    Switch();
    Dir_Err = AD_Left - AD_Right;

    /*分段式P*/
    uint8_t is_turning = 0;
    if (Dir_Err > 3000 * fun || Dir_Err < -3000 * fun) { // 转弯分段
      Kp = Kp_t_;
      is_turning = 1;
    } else { // 直行分段
      Kp = Kp_s;
      is_turning = 0;
    }

    /*改进的PD控制*/
    float pd_output = ImprovedDirectionPDControl(Dir_Err, is_turning);
    Dir_Output = Steer_PWM_Center + pd_output; // 舵机PD修正

    // 限制舵机输出范围
    if (Dir_Output > Steer_PWM_Limit_Right) {
      Dir_Output = Steer_PWM_Limit_Right;
    } else if (Dir_Output < Steer_PWM_Limit_Lift) {
      Dir_Output = Steer_PWM_Limit_Lift;
    }
    SteerPWM = (uint16_t)Dir_Output;

    // 差速PD控制
    Dir_Err_Wheel = ImprovedWheelPDControl(SteerPWM, Dir_Err, is_turning);

    // 4. 电机控制逻辑
    if (Switch_EN_ON) { // 如果使能开关打开
      GPIO_PinState raw_reed_state =HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3); 
      GPIO_PinState filtered_reed_state =ReedFilter(raw_reed_state); 
      if (filtered_reed_state ==GPIO_PIN_SET) { // 当干簧管吸合时（被磁铁靠近，输入为高电平）
        reed_count++;    
      }
      if (reed_count >= 30) {                     // 30需调
        Wheel_Left_Speed = Wheel_Right_Speed = 0; // 停车
        HAL_GPIO_WritePin(Enable_IO_GPIO_Port, Enable_IO_Pin, GPIO_PIN_RESET);
      } else {
        if (stop < 10) { // 10个周期的启动延时（100ms）
          stop++;
          Wheel_Left_Speed = 0;
          heel_Right_Speed = 0;
        }else {     // 启动延时完成，正常行驶
            Wheel_Left_Speed = Std_Speed;                    // 设定速度
            Wheel_Right_Speed = (int)Wheel_Left_Speed * 1.2; // 1.45
          }
        } 
        Wheel_Left_Speed -= (int)(Dir_Err_Wheel * 0.7); // 差速修正
        Wheel_Right_Speed += (int)(Dir_Err_Wheel * 0.82);
        HAL_GPIO_WritePin(Enable_IO_GPIO_Port, Enable_IO_Pin, GPIO_PIN_SET);
    } else {
      Wheel_Left_Speed = 0; // 使能关闭，停止
      Wheel_Right_Speed = 0;
      HAL_GPIO_WritePin(Enable_IO_GPIO_Port, Enable_IO_Pin, GPIO_PIN_RESET);
    }

    // 6. 速度限幅
    Wheel_Left_Speed = Data_Limit(Wheel_Left_Speed, -4000, 4000);
    Wheel_Right_Speed = Data_Limit(Wheel_Right_Speed, -4000, 4000);

    // 7. 左电机PWM和方向控制
    if (Wheel_Left_Speed >= 0) {
      Wheel_Left_PWM = Wheel_Left_Speed;
      Wheel_Left_IO = GPIO_PIN_RESET;
    } else {
      Wheel_Left_PWM = 4800 + Wheel_Left_Speed;
      Wheel_Left_IO = GPIO_PIN_SET;
    }

    // 8. 右电机PWM和方向控制
    if (Wheel_Right_Speed >= 0) {
      Wheel_Right_PWM = Wheel_Right_Speed;
      Wheel_Right_IO = GPIO_PIN_RESET;
    } else {
      Wheel_Right_PWM = 4800 + Wheel_Right_Speed;
      Wheel_Right_IO = GPIO_PIN_SET;
    }

    // 9. 输出控制信号
    if ((Dir_Err > 2000 * fun && Dir_Err < 3000 * fun) ||(Dir_Err < -2000 * fun && Dir_Err > -3000 * fun)) { // 转弯分段
      Data_Limit(SteerPWM, Steer_PWM_Center - 40, Steer_PWM_Center + 40);
      PWM_SetDuty(PWM_TIM3_CH3_B0, SteerPWM);
    } else if (Dir_Err > 3000 * fun) {
      PWM_SetDuty(PWM_TIM3_CH3_B0, 845);
    } else if (Dir_Err < -3000 * fun) {
      PWM_SetDuty(PWM_TIM3_CH3_B0, 745);
    } else {
      SteerPWM = Steer_PWM_Center;
      Data_Limit(SteerPWM, Steer_PWM_Center - 7, Steer_PWM_Center + 7);
      PWM_SetDuty(PWM_TIM3_CH3_B0, SteerPWM); // 舵机控制
    }
    PWM_SetDuty(PWM_TIM4_CH2_B7, Wheel_Left_PWM);  // 左电机PWM
    PWM_SetDuty(PWM_TIM4_CH1_B6, Wheel_Right_PWM); // 右电机PWM
    HAL_GPIO_WritePin(Wheel_Left_IO_GPIO_Port, Wheel_Left_IO_Pin,Wheel_Left_IO);
    HAL_GPIO_WritePin(Wheel_Right_IO_GPIO_Port, Wheel_Right_IO_Pin,Wheel_Right_IO);

    // 11. 串口调试输出
    VOFA_JustFloat(&huart_DBG);

    HAL_Delay(10); // 10ms循环周期
  }
}

/*拨码开关*/
void Switch(void) {
  // 读取开关状态，将其作为二进制数处理
  Switch_GetCode();
  uint8_t bit1 = Switch_GetState_Index(Switch_Index_1);
  uint8_t bit2 = Switch_GetState_Index(Switch_Index_2);
  uint8_t bit3 = Switch_GetState_Index(Switch_Index_3);
  uint8_t bit4 = Switch_GetState_Index(Switch_Index_4);
  uint8_t switch_code = (bit4 << 3) | (bit3 << 2) | (bit2 << 1) | (bit1 << 0);

  if (switch_code != last_switch_count) {
    last_switch_count = switch_code;
    // 根据二进制编码设置参数
    switch (switch_code) {
    case 0b0000: // 全部开关关闭 (0000)
      Std_Speed = 480;
      Kp_s = 0.05;  // 直行P参数
      Kp_t_ = 0.40; // 转弯P参数
      Kd_s = 0.15;  // 直行D参数
      Kd_t_ = 0.20; // 转弯D参数
      Kd = 0.2;     // 兼容原参数
      fun = 1;
      break;

    case 0b0001: // 仅开关1激活 (0001)
      Std_Speed = 550;
      Kp_s = 0.04;
      Kp_t_ = 0.51;
      Kd_s = 0.15;
      Kd_t_ = 0.15;
      Kd = 0.15;
      fun = 0.6;
      break;

    case 0b0010: // 仅开关2激活 (0010)
      Std_Speed = 650;
      Kp_s = 0.06;
      Kp_t_ = 0.51;
      Kd_s = 0.20;
      Kd_t_ = 0.23;
      Kd = 0.23;
      fun = 0.7;
      break;

    case 0b0100: // 仅开关3激活 (0100)
      Std_Speed = 650;
      Kp_s = 0.02;
      Kp_t_ = 0.45;
      Kd_s = 0.20;
      Kd_t_ = 0.25;
      Kd = 0.25;
      fun = 0.9;
      break;

    case 0b1000: // 仅开关4激活 (1000)
      Std_Speed = 650;
      Kp_s = 0.05;
      Kp_t_ = 0.62;
      Kd_s = 0.20;
      Kd_t_ = 0.235;
      Kd = 0.235;
      break;

    default:
      switch_code = 0b0001; // 默认参数
      break;
    }
    PWM_SetDuty(PWM_TIM1_CH4_A11,
                Switch_GetState_Index(Switch_Index_1) * PWM_BreathDuty());
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin,
                      Switch_GetState_Index(Switch_Index_2));
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin,
                      Switch_GetState_Index(Switch_Index_3));
    HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin,
                      Switch_GetState_Index(Switch_Index_4));
  }
}

/**
 * @brief 改进的方向环PD控制算法
 * @param error 方向误差
 * @param is_turning 是否处于转弯状态
 * @return 控制输出
 */
float ImprovedDirectionPDControl(float error, uint8_t is_turning) {
  // 根据运行状态选择参数
  float kp = is_turning ? Kp_t_ : Kp_s;
  float kd = is_turning ? Kd_t_ : Kd_s;

  // 计算微分项（使用固定时间步长10ms = 0.01s）
  float dt = 0.01f;
  float derivative = CalculateFilteredDerivative(error, dt);

  // PD控制输出
  float output = kp * error + kd * derivative;

  return output;
}

/**
 * @brief 改进的差速PD控制
 * @param steer_output 舵机输出值
 * @param error 方向误差
 * @param is_turning 是否处于转弯状态
 * @return 差速输出
 */
float ImprovedWheelPDControl(float steer_output, float error,uint8_t is_turning) {
  // 计算舵机偏差
  float steer_error = steer_output - Steer_PWM_Center;

  // 根据运行状态选择参数
  float kp = is_turning ? Kp_t_ : Kp_s;
  float kd = is_turning ? Kd_t_ * 3.0f : Kd_s * 2.0f; // 差速微分项适当调整

  // 计算微分项
  float dt = 0.01f;
  float derivative = CalculateFilteredDerivative(error, dt);

  // 差速PD控制输出
  float wheel_output = kp * steer_error + kd * derivative;

  // 限制差速输出范围
  if (wheel_output > Dir_Err_Wheel_Limit_High) {
    wheel_output = Dir_Err_Wheel_Limit_High;
  } else if (wheel_output < Dir_Err_Wheel_Limit_Low) {
    wheel_output = Dir_Err_Wheel_Limit_Low;
  }

  return wheel_output;
}

/**
 * @brief 改进的微分项计算，带滤波功能
 * @param current_error 当前误差
 * @param dt 时间间隔
 * @return 滤波后的微分值
 */
float CalculateFilteredDerivative(float current_error, float dt) {
  // 计算原始微分
  float raw_derivative = (current_error - last_error) / dt;

  // 一阶低通滤波器参数
  const float filter_coeff = 0.2f; // 滤波系数，越小滤波越强

  // 应用一阶低通滤波
  filtered_derivative =
      filter_coeff * raw_derivative + (1.0f - filter_coeff) * last_derivative;

  // 更新历史值
  last_error = current_error;
  last_derivative = filtered_derivative;

  return filtered_derivative;
}

/**
 * @brief 电感值滑动窗口滤波
 * @param raw_value 原始ADC值
 * @param window 滑动窗口数组
 * @param window_size 窗口大小
 * @param index 窗口索引
 * @return 滤波后的电感值
 */
int16_t InductorFilter(int16_t raw_value, int16_t *window, int window_size,int *index) {
  window[*index] = raw_value; // 将当前值存入滑动窗口
  *index = (*index + 1) % window_size;

  // 计算窗口内所有值的平均值
  int32_t sum = 0;
  for (int i = 0; i < window_size; i++) {
    sum += window[i];
  }

  return (int16_t)(sum / window_size);
}

/**
 * @brief 干簧管状态滑动窗口滤波
 * @param current_state 当前干簧管状态
 * @return 滤波后的干簧管状态
 */
GPIO_PinState ReedFilter(GPIO_PinState current_state) {
  // 将当前状态存入滑动窗口
  reed_window[reed_window_index] = current_state;
  reed_window_index = (reed_window_index + 1) % REED_WINDOW_SIZE;

  // 统计窗口中高电平的数量
  int high_count = 0;
  for (int i = 0; i < REED_WINDOW_SIZE; i++) {
    if (reed_window[i] == GPIO_PIN_SET) { // 高电平
      high_count++;
    }
  }

  // 如果超过一半为高电平，则认为检测到磁铁
  if (high_count > REED_WINDOW_SIZE / 2) {
    return GPIO_PIN_SET; // 高电平，检测到磁铁
  } else {
    return GPIO_PIN_RESET; // 低电平，未检测到磁铁
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
