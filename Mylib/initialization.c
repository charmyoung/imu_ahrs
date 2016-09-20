#include "main.h"

void  Initialization(void)
{
	
		int i = 0;//���ڳ�ʼ��mpu6050��hmc5883l
		SystemInit();//ϵͳʱ������
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//NVIC���ȼ���2
		
		Led_Configuration();//LED��ʼ��������mpu6050��hmc5883l�ĳ�ʼ����Debug
		
		USART1_Config();//����1�����ã�DBUS RX��
		USART3_Configuration();//����3�����ã����ڵ��ԣ�
		
		TIM2_Configuration();//���ڼ�ʱ
	
		CAN1_Configuration();//	Can1�����ã����ڿ�����̨
		CAN2_Configuration();//Can2�����ã����ڿ��Ƶ���	

		delay_ms(500);    
		
		while(MPU6050_Initialization() == 0xff || HMC5883L_Initialization() == 0xff) 
    {
        i++;     //���һ�γ�ʼ��û�гɹ����Ǿ�����һ��                     
        if(i>10) //�����ʼ��һֱ���ɹ����Ǿ�ûϣ���ˣ�������ѭ������ɫLEDһֱ��
        {
            while(1) 
            {
                LED_GREEN_TOGGLE();
                delay_ms(50);
                
            }
        }  
    } 
		
    MPU6050_Gyro_calibration();//MPU6050У׼
    MPU6050_HMC5883L_Interrupt_Configuration(); //HMC��MPU���ж�����
		
		PWM_Configuration();//PWM���ã����ڿ���Ħ���ֺͲ������
		delay_ms(500);
		
		#ifdef RM35
		CAN_RoboModule_DRV_Reset(0,0);//CAN����
		delay_ms(1000);
		CAN_RoboModule_DRV_Mode_Choice(0,0,PWM_VELOCITY_MODE);//PWM�ٶ�ģʽ
		delay_ms(1000);
		#endif
		
		//TIM6_Configuration();//��ʱ�������ã�һ�����ڲ��ԡ�
		//TIM6_Start();//��ʱ��������һ�����ڲ���
		//Cmd_Reset();��̨У׼��һ�㲻�ã�
}
