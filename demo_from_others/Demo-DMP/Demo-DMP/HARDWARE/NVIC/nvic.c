#include "stm32f10x.h"
#include "misc.h"
#include "nvic.h"
/*
中断分配说明，优先级由高向低排
顺序       类型            作用                   抢占优先级      响应优先级
 1         USART1       串口通信中断                  3                3

*/


//1----串口1通信
void USART1_NVIC_Config(NVIC_InitTypeDef NVIC_InitStructure)
{
	  //Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
}

void NVIC_Config(u8 Interrupt_flag)
{

  	NVIC_InitTypeDef   NVIC_InitStructure;
	
	/* 
	配置中断使用组合2  抢占式3位(0-3)，响应式1位(0-3)
	配置中断使用组合3  抢占式3位(0-7)，响应式1位(0-1) */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	if(Interrupt_flag == 1)
	{
		//串口1通讯 3 3
		USART1_NVIC_Config(NVIC_InitStructure);
	}


}


