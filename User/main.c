#include "main.h"

char buffer1[32];//���ڴ���printf


int main(void)
{   
		
		int i = 0;//���ڳ�ʼ��mpu6050��hmc5883l
		SystemInit();//ϵͳʱ������
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//NVIC���ȼ���2
		
		Led_Configuration();//LED��ʼ��������mpu6050��hmc5883l�ĳ�ʼ����Debug
		
		USART1_Config();//����1�����ã�DBUS RX��
		USART3_Configuration();//����3�����ã����ڵ��ԣ�
		TIM2_Configuration();//���ڼ�ʱ
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
		
		while(1)
		{		
			
			
			//���ڲ�������
			//delay_ms(50);
			//sprintf(buffer1,"%d \r %d \r %f\n",Pitch.targetAngle,Pitch.thisAngle,MPU6050_Real_Data.Gyro_Y );
			//printf(buffer1);
			
		}
}
