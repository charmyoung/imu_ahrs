#ifndef __USART3_H__
#define __USART3_H__
#include "stdint.h"
extern uint8_t TX_ROS_Data[9]; 
#define	TX_ROS_Head 0x55
#define	TX_ROS_Pos_Symbol 0x51



void USART3_Configuration(void);
void USART3_SendChar(unsigned char b);
void USART3_SendInt(int i);
void USART3_SendStr(uint8_t *str);
void USART3_TX_ROS(void);

#endif
