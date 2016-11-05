#include "stm32f10x.h"
#include "misc.h"
#include "nvic.h"
/*
�жϷ���˵�������ȼ��ɸ������
˳��       ����            ����                   ��ռ���ȼ�      ��Ӧ���ȼ�
 1         USART1       ����ͨ���ж�                  3                3

*/


//1----����1ͨ��
void USART1_NVIC_Config(NVIC_InitTypeDef NVIC_InitStructure)
{
	  //Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
}

void NVIC_Config(u8 Interrupt_flag)
{

  	NVIC_InitTypeDef   NVIC_InitStructure;
	
	/* 
	�����ж�ʹ�����2  ��ռʽ3λ(0-3)����Ӧʽ1λ(0-3)
	�����ж�ʹ�����3  ��ռʽ3λ(0-7)����Ӧʽ1λ(0-1) */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	if(Interrupt_flag == 1)
	{
		//����1ͨѶ 3 3
		USART1_NVIC_Config(NVIC_InitStructure);
	}


}


