#include "Fun.h"



void VOFA_JustFloat(UART_HandleTypeDef *huart)
{
    float data[VOFA_JustFloat_index] = {0.0,0,0,0,0};
    const uint8_t end[4] = {0x00, 0x00, 0x80, 0x7f};

    extern int16_t Wheel_Left_Speed, Wheel_Right_Speed;
		extern int16_t Std_Speed;
		extern uint16_t SteerPWM;
    
    data[0] = Wheel_Left_Speed;
    data[1] = Wheel_Right_Speed;
		data[2] = Std_Speed;
		data[3] = SteerPWM;
		data[4] = 0;
		data[5] = 0;

    Usart_SendArr(huart, (uint8_t*)data, sizeof(float) * VOFA_JustFloat_index);

    Usart_SendArr(huart, (uint8_t*)end, 4);
}


void MasterPC_send(UART_HandleTypeDef *huart)
{
    signed int SendData_Temp = 0;
    unsigned char SendData_Temp_High,SendData_Temp_Low;

    Usart_SendByte(huart,0xFF);
    Usart_SendByte(huart,0x00);

    //����1
//    SendData_Temp = (signed int)PID_body_Omega.pOut;
    SendData_Temp_High = SendData_Temp >> 8;
    SendData_Temp_Low = SendData_Temp & 0xff;
    Usart_SendByte(huart,SendData_Temp_Low);
    Usart_SendByte(huart,SendData_Temp_High);

    //����2
//    SendData_Temp = (signed int)PID_body_Omega.iOut;
    SendData_Temp_High = SendData_Temp >> 8;
    SendData_Temp_Low = SendData_Temp & 0xff;
    Usart_SendByte(huart,SendData_Temp_Low);
    Usart_SendByte(huart,SendData_Temp_High);

    //����3
//    SendData_Temp = (signed int)PID_body_Angle.pOut;
    SendData_Temp_High = SendData_Temp >> 8;
    SendData_Temp_Low = SendData_Temp & 0xff;
    Usart_SendByte(huart,SendData_Temp_Low);
    Usart_SendByte(huart,SendData_Temp_High);

    //����4
//    SendData_Temp = (signed int)PID_body_Angle.iOut;
    SendData_Temp_High = SendData_Temp >> 8;
    SendData_Temp_Low = SendData_Temp & 0xff;
    Usart_SendByte(huart,SendData_Temp_Low);
    Usart_SendByte(huart,SendData_Temp_High);

    //����5
//    SendData_Temp = (signed int)PID_body_Angle.dOut;
    SendData_Temp_High = SendData_Temp >> 8;
    SendData_Temp_Low = SendData_Temp & 0xff;
    Usart_SendByte(huart,SendData_Temp_Low);
    Usart_SendByte(huart,SendData_Temp_High);

//    //����6
//    SendData_Temp = (signed int)PID_body_Omega.out;
//    SendData_Temp_High = SendData_Temp >> 8;
//    SendData_Temp_Low = SendData_Temp & 0xff;
//    Usart_SendByte(&huart4,SendData_Temp_Low);
//    Usart_SendByte(&huart4,SendData_Temp_High);

//    //����7
//    SendData_Temp = (signed int)giGyro_Z;
//    SendData_Temp_High = SendData_Temp >> 8;
//    SendData_Temp_Low = SendData_Temp & 0xff;
//    Usart_SendByte(&huart4,SendData_Temp_Low);
//    Usart_SendByte(&huart4,SendData_Temp_High);

//    //����8
//    SendData_Temp = (signed int)0;
//    SendData_Temp_High = SendData_Temp >> 8;
//    SendData_Temp_Low = SendData_Temp & 0xff;
//    Usart_SendByte(&huart4,SendData_Temp_Low);
//    Usart_SendByte(&huart4,SendData_Temp_High);

    Usart_SendByte(huart,0xAA);
    Usart_SendByte(huart,0x55);
}






