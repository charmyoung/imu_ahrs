#include "main.h"

/*----CAN1_TX-----PA12----*/
/*----CAN1_RX-----PA11----*/


void CAN1_Configuration(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);

    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &gpio);
    
    nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);    
    
    CAN_DeInit(CAN1);
    CAN_StructInit(&can);
    
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = DISABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = ENABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_9tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN1, &can);

	can_filter.CAN_FilterNumber=0;
	can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh=0x0000;
	can_filter.CAN_FilterIdLow=0x0000;
	can_filter.CAN_FilterMaskIdHigh=0x0000;
	can_filter.CAN_FilterMaskIdLow=0x0000;
	can_filter.CAN_FilterFIFOAssignment=0;
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);
    
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 
}



static uint16_t Can1_CNT = 0;//CAN�����жϴ���

/*
//���ڵ���������δ���á�
//����жϿ�������1ms����ú���һ��

#define CurrTempNUM 100 //���ֵ�ʱ������
#define CurrLimit 13000 //��������12000 for 6025


static int16_t CurrTemp_205[CurrTempNUM] = {0};
static int16_t CurrTemp_206[CurrTempNUM] = {0};
static int16_t CurrCnt_205 = 0;
static int16_t CurrCnt_206 = 0;
static int32_t CurrInt_205 =0;
static int32_t CurrInt_206 =0;
volatile unsigned char OverCurr_flag = 0;

void CurrentProtect(void)//���ڵ�����������
{
    unsigned int i;
    unsigned int j;
    int32_t currtemp_205 = 0;
    int32_t currtemp_206 = 0;
    if(Can1_CNT%20 == 0)
    {
        // 205��������
        CurrTemp_205[CurrCnt_205] = Yaw.targetCurrent;
        CurrCnt_205++;
        
        if(CurrCnt_205 >= CurrTempNUM){
            CurrCnt_205 = 0;
        }            
        for(i = 0;i < CurrTempNUM;i++)
        {
            currtemp_205 += CurrTemp_205[i];
            CurrInt_205 = currtemp_205/CurrTempNUM;
        }
        // 206��������
        CurrTemp_206[CurrCnt_206] = Pitch.targetCurrent;
        CurrCnt_206++;
        
        if(CurrCnt_206 >= CurrTempNUM){
            CurrCnt_206 = 0;
        }            
        for(j = 0;j < CurrTempNUM;j++)
        {
            currtemp_206 += CurrTemp_206[j];
            CurrInt_206 = currtemp_206/CurrTempNUM;
        }
    }
        
    if((abs(CurrInt_206) > CurrLimit)||(abs(CurrInt_205) > CurrLimit))
        {
            OverCurr_flag = 1;
        }

}
*/

/*************************************************************************
                          CAN1_TX_IRQHandler
������CAN1�ķ����жϺ���
*************************************************************************/

unsigned char can_tx_success_flag=0;

void CAN1_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
	{
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
		 can_tx_success_flag=1;
		
    }
}

/*************************************************************************
                          CAN1_RX0_IRQHandler

Description��  	��̨�����CAN���ݽ����ж�
*************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;    
    
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
	{
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx_message);       
             
			if(rx_message.StdId == 0x205)
        {             
					 
				
             //�����̨���0x205������ֵ 
							Yaw.thisAngle_Raw = (rx_message.Data[0]<<8)|rx_message.Data[1];     //��е�Ƕ�
							Yaw.thisCurrent = (rx_message.Data[2]<<8)|rx_message.Data[3];      //ʵ�ʵ���ֵ
							Yaw.targetCurrent = (rx_message.Data[4]<<8)|rx_message.Data[5];     //�����ĵ���ֵ
							Yaw.thisAngle=Yaw.thisAngle_Raw;
							if((Yaw.thisAngle & 0x1000) == 0x1000)
							{
								Yaw.thisAngle = Yaw.thisAngle_Raw - 8191 ;   //��205��ֵ��Ϊ������������
							}  		
							Yaw.thisAngle = 3000-Yaw.thisAngle;
        }	
        if(rx_message.StdId == 0x206)
        { 
             //�����̨���0x206������ֵ  
							Pitch.thisAngle_Raw = (rx_message.Data[0]<<8)|rx_message.Data[1];     //��е�Ƕ�
							Pitch.thisCurrent = (rx_message.Data[2]<<8)|rx_message.Data[3];      //ʵ�ʵ���ֵ
							Pitch.targetCurrent = (rx_message.Data[4]<<8)|rx_message.Data[5];     //�����ĵ���ֵ
							Pitch.thisAngle=Pitch.thisAngle_Raw;
							if((Pitch.thisAngle & 0x1000) == 0x1000)
							{
								Pitch.thisAngle = Pitch.thisAngle_Raw - 8191;   //��205��ֵ��Ϊ������������
							}  
							Pitch.thisAngle = 3000-Pitch.thisAngle;
        }
        Can1_CNT++;		
    }
}
/********************************************************************************
Name��          void Cmd_ESC(int16_t current_205,int16_t current_206)

Description��  	������巢��ָ����������Ƶ����ID��Ϊ0x1FF��ֻ����������壬
                  ���ݻش�IDΪ0x205��0x206
*********************************************************************************/
void Cmd_ESC(int16_t current_205,int16_t current_206)
{
    CanTxMsg tx_message;
    
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (unsigned char)(current_205 >> 8);
    tx_message.Data[1] = (unsigned char)current_205;
    tx_message.Data[2] = (unsigned char)(current_206 >> 8);
    tx_message.Data[3] = (unsigned char)current_206;
    tx_message.Data[4] = NULL;
		tx_message.Data[5] = NULL;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = NULL;
    
    CAN_Transmit(CAN1,&tx_message);
}

/********************************************************************************
Name��          void Cmd_Reset(void)

Description��  	������巢��ָ������У�������,ID��Ϊ0x1FF,��6��byteΪ0x04
*********************************************************************************/
void Cmd_Reset(void)
{
    CanTxMsg tx_message;
    
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x00;
    tx_message.Data[2] = 0x00;
    tx_message.Data[3] = 0x00;
    tx_message.Data[4] = 0x00;
		tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x04;
    tx_message.Data[7] = 0x00;
    
    CAN_Transmit(CAN1,&tx_message);
}






