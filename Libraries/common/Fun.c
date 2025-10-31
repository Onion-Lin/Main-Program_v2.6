#include "Fun.h"



void VOFA_JustFloat(UART_HandleTypeDef *huart)
{
    float data[VOFA_JustFloat_index] = {0.0};
    const uint8_t end[4] = {0x00, 0x00, 0x80, 0x7f};

    extern int16_t AD_Left, AD_Right;
    
    data[0] = AD_Left;
    data[1] = AD_Right;

    Usart_SendArr(huart, (uint8_t*)data, sizeof(float) * VOFA_JustFloat_index);

    Usart_SendArr(huart, (uint8_t*)end, 4);
}


void MasterPC_send(UART_HandleTypeDef *huart)
{
    signed int SendData_Temp = 0;
    unsigned char SendData_Temp_High,SendData_Temp_Low;

    Usart_SendByte(huart,0xFF);
    Usart_SendByte(huart,0x00);

    //曲线1
//    SendData_Temp = (signed int)PID_body_Omega.pOut;
    SendData_Temp_High = SendData_Temp >> 8;
    SendData_Temp_Low = SendData_Temp & 0xff;
    Usart_SendByte(huart,SendData_Temp_Low);
    Usart_SendByte(huart,SendData_Temp_High);

    //曲线2
//    SendData_Temp = (signed int)PID_body_Omega.iOut;
    SendData_Temp_High = SendData_Temp >> 8;
    SendData_Temp_Low = SendData_Temp & 0xff;
    Usart_SendByte(huart,SendData_Temp_Low);
    Usart_SendByte(huart,SendData_Temp_High);

    //曲线3
//    SendData_Temp = (signed int)PID_body_Angle.pOut;
    SendData_Temp_High = SendData_Temp >> 8;
    SendData_Temp_Low = SendData_Temp & 0xff;
    Usart_SendByte(huart,SendData_Temp_Low);
    Usart_SendByte(huart,SendData_Temp_High);

    //曲线4
//    SendData_Temp = (signed int)PID_body_Angle.iOut;
    SendData_Temp_High = SendData_Temp >> 8;
    SendData_Temp_Low = SendData_Temp & 0xff;
    Usart_SendByte(huart,SendData_Temp_Low);
    Usart_SendByte(huart,SendData_Temp_High);

    //曲线5
//    SendData_Temp = (signed int)PID_body_Angle.dOut;
    SendData_Temp_High = SendData_Temp >> 8;
    SendData_Temp_Low = SendData_Temp & 0xff;
    Usart_SendByte(huart,SendData_Temp_Low);
    Usart_SendByte(huart,SendData_Temp_High);

//    //曲线6
//    SendData_Temp = (signed int)PID_body_Omega.out;
//    SendData_Temp_High = SendData_Temp >> 8;
//    SendData_Temp_Low = SendData_Temp & 0xff;
//    Usart_SendByte(&huart4,SendData_Temp_Low);
//    Usart_SendByte(&huart4,SendData_Temp_High);

//    //曲线7
//    SendData_Temp = (signed int)giGyro_Z;
//    SendData_Temp_High = SendData_Temp >> 8;
//    SendData_Temp_Low = SendData_Temp & 0xff;
//    Usart_SendByte(&huart4,SendData_Temp_Low);
//    Usart_SendByte(&huart4,SendData_Temp_High);

//    //曲线8
//    SendData_Temp = (signed int)0;
//    SendData_Temp_High = SendData_Temp >> 8;
//    SendData_Temp_Low = SendData_Temp & 0xff;
//    Usart_SendByte(&huart4,SendData_Temp_Low);
//    Usart_SendByte(&huart4,SendData_Temp_High);

    Usart_SendByte(huart,0xAA);
    Usart_SendByte(huart,0x55);
}






