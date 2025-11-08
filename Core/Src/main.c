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
int16_t AD_Left, AD_Right,raw_ad_left,raw_ad_right;            // 左右电感的ADC值
float Dir_Err;                        // 方向偏差
int16_t Dir_Output;                   // 方向输出值
uint16_t SteerPWM = Steer_PWM_Center; // 舵机PWM输出值
void SystemClock_Config(void);

// 拨码开关状态缓存
static uint8_t last_switch_count = 0xFF; // 初始化为无效状态
static uint8_t current_switch_count = 0;

// PD与差速变量
float Dir_Err_Wheel = 0;
float Dir_Err_Wheel_Limit_Low = -200; // 差速限幅
float Dir_Err_Wheel_Limit_High = 200;
float Kp, Kp_s, Kp_t_, Kd, Kd_s, Kd_t_; // 直行和转弯的微分参数
int16_t fliter_Buf_L[3] = {0, 0, 0};    // 左电感滑动窗口
int16_t fliter_Buf_R[3] = {0, 0, 0};    // 右电感滑动窗口
int16_t Kd_Change;
float Err_Tmp[3] = {0, 0}; // 历史误差
//double fun;
static float last_error = 0;          // 上一次误差
static float Last_Kd_Velue = 0;     // 上一次微分值
static float Filtered_Kd_Velue = 0; // 滤波后的微分值

// 干簧管停车相关变量
int reed_count = 0; // 干簧管计数器
int middle = 0;     // 直行转弯标志
int stop_time=0;
int parking_state = 0;        // 停车状态：0=正常行驶，1=检测到停车信号，2=延时行驶，3=已停车
uint32_t parking_start_time = 0; // 停车开始时间

// 左电机驱动变量
int16_t Wheel_Left_Speed;      // 驱动电机速度期望
int16_t Std_Speed;             // 基准速度
uint16_t Wheel_Left_PWM = 0;   // 驱动电机PWM输出值
uint8_t Wheel_Left_IO = RESET; // 驱动电机IO口电平

// 右电机驱动变量
int16_t Wheel_Right_Speed;      // 驱动电机速度期望
uint16_t Wheel_Right_PWM = 0;   // 驱动电机PWM输出值
uint8_t Wheel_Right_IO = RESET; // 驱动电机IO口电平

// 转弯状态检测相关
static uint8_t turn_state = 0;  // 0=直行，1=小转弯，2=大转弯
static uint32_t turn_start_time = 0; // 转弯开始时间
static uint8_t turn_hysteresis_counter = 0; // 转弯迟滞计数器

// 发车延时相关
static uint8_t start_delay_enabled = 0;  // 发车延时使能标志
static uint32_t start_delay_start_time = 0; // 发车延时开始时间
static uint8_t start_delay_complete = 0; // 发车延时完成标志

// 电感检测停车相关
static uint8_t ad_detection_stop = 0; // 电感检测停车标志

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
  Last_Kd_Velue = 0;
  Filtered_Kd_Velue = 0;
	uint8_t new_turn_state = 0;

  // 初始化干簧管滤波
  for (int i = 0; i < Stop_Fliter_SIZE; i++) {
    reed_window[i] = GPIO_PIN_RESET;
  }
  reed_window_index = 0;

  // 初始化电感滤波
  for (int i = 0; i < ADC_Fliter_SIZE; i++) {
    Fliter_Left_Window[i] = 0;
    Fliter_Right_Window[i] = 0;
  }
  Pins_window_index = 0;

  while (1) {
    // 1. 读取左右电感值并滤波
    raw_ad_left = ADC_GetValue_Channel(ADC1_CH04_PA4); // A4:R72-L10
    raw_ad_right =ADC_GetValue_Channel(ADC1_CH15_PC5); // C5:R1200-L1300
    AD_Left = ADC_Filter(raw_ad_left, Fliter_Left_Window,ADC_Fliter_SIZE, &Pins_window_index);
    AD_Right = ADC_Filter(raw_ad_right, Fliter_Right_Window,ADC_Fliter_SIZE, &Pins_window_index);
    //AD_Left = ADC_GetValue_Channel(ADC1_CH04_PA4);   // A4:R72-L10
    //AD_Right = ADC_GetValue_Channel(ADC1_CH15_PC5);  // C5:R1200-L1300
    //AD_Left *= fun;
    //AD_Right *= fun;

    // 2.计算输出，PID
    Switch();
		// 检查是否开启使能开关，如果是则开始延时发车
    if (Switch_EN_ON && !start_delay_enabled && !start_delay_complete) {
      start_delay_enabled = 1;
      start_delay_start_time = HAL_GetTick();
      ad_detection_stop = 0; // 重置电感检测停车标志
    }
    
    // 检查是否需要延时发车
    if (Switch_EN_ON && start_delay_enabled && !start_delay_complete) {
      if (HAL_GetTick() - start_delay_start_time >= 1000) { // 1秒延时
        start_delay_complete = 1; // 延时完成
      } else {
        HAL_Delay(10);
        continue;//直接跳进下一次循环
      }
    }
    
    // 检查电感值是否都小于阈值，如果是则停车
    //if (Switch_EN_ON && (AD_Left < AD_THRESHOLD && AD_Right < AD_THRESHOLD)) {
		if (AD_Left < AD_THRESHOLD && AD_Right < AD_THRESHOLD) {
      ad_detection_stop = 1; // 冲出赛道，停车
    }else {
      ad_detection_stop = 0; // 正常行驶
    }
    
    // 电感检测停车处理
    if (ad_detection_stop) {
      Dir_Err = 0; // 将误差设置为0
    } else {
      Dir_Err = ((float)AD_Left - (float)AD_Right)/((float)AD_Left+(float)AD_Right)*120.0; // 差比和
    }
    
    // 转弯状态检测与分段控制
    /*if (Dir_Err > TURN_THRESHOLD_HIGH || Dir_Err < -TURN_THRESHOLD_HIGH) {
      new_turn_state = 2; // 大转弯
    } else if (Dir_Err > TURN_THRESHOLD_LOW || Dir_Err < -TURN_THRESHOLD_LOW) {
      new_turn_state = 1; // 小转弯
    } else {
      new_turn_state = 0; // 直行
    }
    
    // 使用迟滞机制防止频繁切换
    if (new_turn_state != turn_state) {
      turn_hysteresis_counter++;
      if (turn_hysteresis_counter >= 2) { // 连续2次检测到新状态才切换
        turn_state = new_turn_state;
        turn_hysteresis_counter = 0;
        turn_start_time = HAL_GetTick(); // 记录转弯开始时间
      }
    } else {
      turn_hysteresis_counter = 0; // 重置计数器
    }*/
    
		if (Dir_Err > TURN_THRESHOLD_HIGH || Dir_Err < -TURN_THRESHOLD_HIGH) {
      turn_state = 2; // 大转弯
    } else if (Dir_Err > TURN_THRESHOLD_LOW || Dir_Err < -TURN_THRESHOLD_LOW) {
      turn_state = 1; // 小转弯
    } else {
      turn_state = 0; // 直行
    }
    uint8_t is_turning = (turn_state > 0) ? 1 : 0;
    
    // 根据转弯状态设置参数
    if (turn_state == 2) { // 大转弯
      Kp = Kp_t_*2.0;
      Kd = Kd_t_*1.2;
    } else if (turn_state == 1) { // 小转弯
      Kp = Kp_t_; 
      Kd = Kd_t_;
    } else { // 直行
      Kp = Kp_s;
      Kd = Kd_s;
    }

    float pd_output = PD_Control(Dir_Err, is_turning);
    Dir_Output = Steer_PWM_Center + pd_output;    // 舵机PD修正

    // 限制舵机输出范围
    if (Dir_Output > Steer_PWM_Limit_Right) {
      Dir_Output = Steer_PWM_Limit_Right;
    } else if (Dir_Output < Steer_PWM_Limit_Lift) {
      Dir_Output = Steer_PWM_Limit_Lift;
    }
    SteerPWM = (uint16_t)Dir_Output;

    // 差速PD控制
    Dir_Err_Wheel = Wheel_PD_Control(SteerPWM, Dir_Err, is_turning);

    // 4. 电机控制逻辑
    if (Switch_EN_ON && start_delay_complete && !ad_detection_stop && !ad_detection_stop) {  // 如果使能开关打开且延时完成且未检测到停车条件
      GPIO_PinState raw_reed_state =HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3); 
			//GPIO_PinState reed_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
      GPIO_PinState filtered_reed_state =Stop_Filter(raw_reed_state); 
      if (filtered_reed_state ==GPIO_PIN_SET) { // 当干簧管吸合时（被磁铁靠近，输入高电平）
        if (parking_state == 0) { // 只在正常行驶状态下计数
          reed_count++;    
          if (reed_count >= stop_time) {
            parking_state = 1; // 进入停车检测状态
            parking_start_time = HAL_GetTick(); // 记录停车开始时间
          }
        }
      }
      // 处理停车逻辑
      if (parking_state == 1) { // 检测到停车信号，进入延时行驶状态
        parking_state = 2; // 进入延时行驶状态
        parking_start_time = HAL_GetTick(); // 记录延时开始时间
      }else if (parking_state == 2) { // 延时行驶状态
        if ((HAL_GetTick() - parking_start_time) >= 500) { // 0.5秒
          parking_state = 3; // 进入停车状态
        }
      } else if (parking_state >= 3) { // 已停车
        Wheel_Left_Speed = 0; // 使能关闭，停止
        Wheel_Right_Speed = 0;
        HAL_GPIO_WritePin(Enable_IO_GPIO_Port, Enable_IO_Pin, GPIO_PIN_RESET);
      } else {  // 发车，正常行驶
        Wheel_Left_Speed = Std_Speed;                    
        Wheel_Right_Speed = (int)(Wheel_Left_Speed * 1.25); // 1.3
        HAL_GPIO_WritePin(Enable_IO_GPIO_Port, Enable_IO_Pin, GPIO_PIN_SET);
      } 
        // 根据转弯状态调整差速修正
      float turn_factor = 1.0;
      if (turn_state == 2) { // 大转弯
          turn_factor = 2.0; // 增加差速
        } else if (turn_state == 1) { // 小转弯
          turn_factor = 1.0; // 正常差速
        }
        Wheel_Left_Speed -= (int)(Dir_Err_Wheel *0.92 * turn_factor); // 差速修正
        Wheel_Right_Speed += (int)(Dir_Err_Wheel  * turn_factor);
    } else {
      Wheel_Left_Speed = 0; // 使能关闭，停止
      Wheel_Right_Speed = 0;
      HAL_GPIO_WritePin(Enable_IO_GPIO_Port, Enable_IO_Pin, GPIO_PIN_RESET);
      parking_state = 0; // 重置停车状态
      reed_count = 0; // 重置计数器
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
    // 根据转弯状态调整舵机输出
    if (turn_state == 1) { // 小转弯
      Data_Limit(SteerPWM, Steer_PWM_Center - 50, Steer_PWM_Center + 50);
      PWM_SetDuty(PWM_TIM3_CH3_B0, SteerPWM);
    } else if (turn_state == 2) { // 大转弯
      Data_Limit(SteerPWM, Steer_PWM_Center - 85, Steer_PWM_Center + 85);
      PWM_SetDuty(PWM_TIM3_CH3_B0, SteerPWM);
    } else { // 直行
			SteerPWM = Steer_PWM_Center;
      //Data_Limit (SteerPWM,Steer_PWM_Center-7,Steer_PWM_Center+7);
      PWM_SetDuty(PWM_TIM3_CH3_B0, SteerPWM);             // 舵机控制
    }
    
    PWM_SetDuty(PWM_TIM4_CH2_B7, Wheel_Left_PWM);  // 左电机PWM
    PWM_SetDuty(PWM_TIM4_CH1_B6, Wheel_Right_PWM); // 右电机PWM
    HAL_GPIO_WritePin(Wheel_Left_IO_GPIO_Port, Wheel_Left_IO_Pin, Wheel_Left_IO);
    HAL_GPIO_WritePin(Wheel_Right_IO_GPIO_Port, Wheel_Right_IO_Pin,Wheel_Right_IO);

    // 11. 串口调试输出
    VOFA_JustFloat(&huart_DBG);

    HAL_Delay(10); // 10ms循环周期
  }
}

/*拨码开关*/
void Switch(void) {
  Switch_GetCode();
  uint8_t bit1 = Switch_GetState_Index(Switch_Index_1);
  uint8_t bit2 = Switch_GetState_Index(Switch_Index_2);
  uint8_t bit3 = Switch_GetState_Index(Switch_Index_3);
  uint8_t bit4 = Switch_GetState_Index(Switch_Index_4);
  uint8_t switch_code = (bit4 << 3) | (bit3 << 2) | (bit2 << 1) | (bit1 << 0);//按位运算

 if (switch_code != last_switch_count) {
    last_switch_count = switch_code;
    // 根据二进制编码设置参数
    switch(switch_code) {
    case 0b0000: // 全部开关关闭 (0000)
      Std_Speed = 550;
      Kp_s = 0.01;      // 增加直行P值，提高稳定性
      Kp_t_ = 0.48;     // 转弯P值，提供足够的转向力
      Kd_s = 0.18;      // 增加直行D值，减少振荡
      Kd_t_ = 0.23;     // 转弯D值，提高转弯稳定性
      stop_time=15;
      break;

    case 0b0001: // 仅开关1激活 (0001)
      Std_Speed = 650;
      Kp_s = 0.06;      // 适中的直行P值
      Kp_t_ = 0.70;     // 转弯P值
      Kd_s = 0.60;      // 适中的直行D值
      Kd_t_ = 1.20;     // 转弯D值
      stop_time=8;
      break;

    case 0b0010: // 仅开关2激活 (0010)
      Std_Speed = 650;
      Kp_s = 0.06;      // 适中的直行P值
      Kp_t_ = 0.70;     // 转弯P值
      Kd_s = 0.60;      // 适中的直行D值
      Kd_t_ = 0.40;     // 转弯D值
      stop_time=12;
      break;

    case 0b0100: // 仅开关3激活 (0100)
      Std_Speed = 800;
      Kp_s = 0.06;      // 高速时降低P值，避免过冲
      Kp_t_ = 0.64;     // 转弯P值
      Kd_s = 0.40;      // 高速时增加D值，提高稳定性
      Kd_t_ = 0.40;     // 转弯D值
      stop_time=9;
      break;

    case 0b1000: // 仅开关4激活 (1000)
      Std_Speed = 1000;
      Kp_s = 0.02;      // 高速时进一步降低P值
      Kp_t_ = 0.60;     // 转弯P值
      Kd_s = 0.25;      // 高速时进一步增加D值
      Kd_t_ = 0.80;     // 转弯D值
      stop_time=7;
      break;

    default:
      switch_code = 0b0001; // 默认参数
      break;
    }
    PWM_SetDuty(PWM_TIM1_CH4_A11,Switch_GetState_Index(Switch_Index_1) * PWM_BreathDuty());
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, Switch_GetState_Index(Switch_Index_2));
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, Switch_GetState_Index(Switch_Index_3));
    HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, Switch_GetState_Index(Switch_Index_4));
  }
}

/**
 * @brief 改进的方向环PD控制算法
 * @param error 方向误差
 * @param is_turning 是否处于转弯状态
 * @return 控制输出
 */
float PD_Control(float error, uint8_t is_turning) {
  float kp = is_turning ? Kp_t_ : Kp_s;//分段
  float kd = is_turning ? Kd_t_ : Kd_s;

  float dt = 0.01f;//Kd运算
  float derivative = Kd_Math(error, dt);

  // 输出Err值
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
float Wheel_PD_Control(float steer_output, float error,uint8_t is_turning) {
  float steer_error = steer_output - Steer_PWM_Center;//偏差值
  float kp = is_turning ? Kp_t_ : Kp_s;
  float kd = is_turning ? Kd_t_ * 2.5 : Kd_s * 1.8; // 差速微分项适当调整

  float dt = 0.01f;
  float derivative = Kd_Math(error, dt);
  float wheel_output = kp * steer_error + kd * derivative;

  // 限幅
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
float Kd_Math(float current_error, float dt) {
  // 防止除零错误
  if (dt <= 0.0f) {
    dt = 0.01f; // 使用默认时间间隔
  }

  float raw_derivative = (current_error - last_error) / dt;
  const float filter_coeff = 0.3f; // 增加滤波系数，减少噪声
  Filtered_Kd_Velue = filter_coeff * raw_derivative + (1.0f - filter_coeff) * Last_Kd_Velue;

  last_error = current_error;
  Last_Kd_Velue = Filtered_Kd_Velue;

  return Filtered_Kd_Velue;
}

/**
 * @brief 电感值滑动窗口滤波
 * @param raw_value 原始ADC值
 * @param window 滑动窗口数组
 * @param window_size 窗口大小
 * @param index 窗口索引
 * @return 滤波后的电感值
 */
int16_t ADC_Filter(int16_t raw_value, int16_t *window, int window_size,int *index) {
  window[*index] = raw_value; 
  *index = (*index + 1) % window_size;
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
GPIO_PinState Stop_Filter(GPIO_PinState current_state) {
  reed_window[reed_window_index] = current_state;
  reed_window_index = (reed_window_index + 1) % Stop_Fliter_SIZE;
  int high_count = 0;
  for (int i = 0; i < Stop_Fliter_SIZE; i++) {
    if (reed_window[i] == GPIO_PIN_SET) { // 高电平，吸合
      high_count++;
    }
  }

  if (high_count > Stop_Fliter_SIZE / 2) {
    return GPIO_PIN_SET; 
  } else {
    return GPIO_PIN_RESET; 
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



 




