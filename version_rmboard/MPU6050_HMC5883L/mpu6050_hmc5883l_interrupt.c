#include "main.h"
uint32_t a=0;
void MPU6050_HMC5883L_Interrupt_Configuration(void)
{
    GPIO_InitTypeDef    gpio;
    NVIC_InitTypeDef    nvic;
    EXTI_InitTypeDef    exti;
 
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,  ENABLE);   
 
	  gpio.GPIO_Pin = GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA, &gpio);
	
		gpio.GPIO_Pin = GPIO_Pin_3;
		GPIO_Init(GPIOA, &gpio);
    
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,GPIO_PinSource3); 
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,GPIO_PinSource4); 
		
    exti.EXTI_Line = EXTI_Line4;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling;//�½����ж�
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);
		
		exti.EXTI_Line = EXTI_Line3;
    EXTI_Init(&exti);
		
    nvic.NVIC_IRQChannel = EXTI4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
		nvic.NVIC_IRQChannel = EXTI3_IRQn;
		NVIC_Init(&nvic);
}

//MPU6050 �ⲿ�жϴ�����
void EXTI4_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line4) == SET)
    {
        //2ms���ж�һ��
        //��ȡMPU6050����,Ϊ��ʹ��̨�Ŀ��Ƹ�ƽ����
        //ʹ��MPU6050�������������Ϊ�ٶȻ�����
        //����ʹ�õ���巵�ػ�е�Ƕ�ֵ���ٶȻ���������������������
        MPU6050_ReadData();//��ȡδ�˲�����                                              
        MPU6050_Data_Filter();//��Ҫ��mpu6050���ٶȼƵľ�ֵ�˲��������ǻ�����ת��
				
			
				/*
				sprintf(buffer1,"%d \n",Get_Time_Micros()-a);
				a=Get_Time_Micros();
				printf(buffer1);
				*/
				
				
				EXTI_ClearFlag(EXTI_Line4);          
				EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

void EXTI3_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line3) == SET)
    {
        //75hz-13ms���ж�һ�Σ�NOT USED����30hz-33ms���ж�һ��
        HMC5883L_ReadData();
				
				//AHRS����
				AHRS_Calculate( 				 Gyro_Radian_Data.X,
																 Gyro_Radian_Data.Y,
																 Gyro_Radian_Data.Z,
																 Accel_Raw_Average_Data.X,
																 Accel_Raw_Average_Data.Y,
																 Accel_Raw_Average_Data.Z,
																 HMC5883L_Real_Data.Mag_X,
																 HMC5883L_Real_Data.Mag_Y,
																 HMC5883L_Real_Data.Mag_Z);
				
				/*
				//sprintf(buffer1,"%d \r %d \r %d \n",HMC5883L_Raw_Data.Mag_X,HMC5883L_Raw_Data.Mag_Y,HMC5883L_Raw_Data.Mag_Z);
				sprintf(buffer1,"%f \n",HMC5883L_Real_Data.Yaw_Angle);
				printf(buffer1);
				delay_ms(100);
				*/
				
				EXTI_ClearFlag(EXTI_Line3);          
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}

