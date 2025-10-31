/*************************************************
Copyright (C), 2016-2023, TYUT JBD TRoMaC
File name: 
Author: 
Version:               
Date: 
Description:  
Others:      
Function List:
History:
<author>    <time>          <version>       <desc>
**************************************************/
#ifndef __DeviceCheck_h
#define __DeviceCheck_h

#include "headfiles.h"

/* =========== 宏定义 ================ */

#define Device_HealthMax        300
#define Device_HealthDefault    200
#define Device_CheckNum         11

typedef enum
{
    // 空设备
    DeviceID_NULL                   = 0x000,
    // 接收机
    DeviceID_DR16                   = 0x001,
    
    // 视觉串口连接
    DeviceID_VisionNUC              ,
    
    // 陀螺仪
    DeviceID_ICM20602               ,
    
    // 电管user口
    DeviceID_Referee                ,
    
    // 超电控制板
    DeviceID_SuperPower             ,
    
    // Can通信设备
    DeviceID_Motor_Pitch            = 0x100,
    DeviceID_Motor_Yaw              ,
    DeviceID_Motor_Fric_Left        ,
    DeviceID_Motor_Fric_Right       ,
    DeviceID_Motor_Launch_Pick      ,
    DeviceID_Motor_Launch_Push      ,
    DeviceID_Motor_LF_Dir           ,
    DeviceID_Motor_RF_Dir           ,
    DeviceID_Motor_LB_Dir           ,
    DeviceID_Motor_RB_Dir           ,
    DeviceID_Motor_LF_Spd           ,
    DeviceID_Motor_RF_Spd           ,
    DeviceID_Motor_LB_Spd           ,
    DeviceID_Motor_RB_Spd           ,
    
    DeviceID_Chassis                ,
    DeviceID_Gimbal                 ,
    
    
    DeviceID_freeRTOS               = 0xE00,
    
                      
}DeviceID_enum;

typedef enum
{
    DeviceState_Offline             = 0,
    DeviceState_Online              = 1,
    
}DeviceState_enum;

typedef struct
{
    DeviceID_enum                   DeviceID;
    DeviceState_enum                DeviceState;
    
    int16_t                         HealthValue;    // 设备健康值
}DeviceCheckHandle_TypeDef;

/* =========== 全局变量声明 ========== */
extern DeviceCheckHandle_TypeDef DeviceList[Device_CheckNum];

/* =========== 函数声明 ============== */
void Device_MonitoringInit(
    uint32_t DeviceID_1 ,
    uint32_t DeviceID_2 ,
    uint32_t DeviceID_3 ,
    uint32_t DeviceID_4 ,
    uint32_t DeviceID_5 ,
    uint32_t DeviceID_6 ,
    uint32_t DeviceID_7 ,
    uint32_t DeviceID_8 ,
    uint32_t DeviceID_9 ,
    uint32_t DeviceID_10,
    uint32_t DeviceID_11);
void Device_OnlineCheck_ALL(void);

void Device_HandleInit(DeviceCheckHandle_TypeDef *DeviceCheckHandle, uint32_t DeviceID);
void Device_Update(DeviceID_enum DeviceID);
DeviceState_enum Device_GetState(DeviceID_enum DeviceID);

#endif /* __DeviceCheck_h */
