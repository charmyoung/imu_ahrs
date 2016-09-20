#include "main.h"

char buffer1[32];//用于串口printf


int main(void)
{   
		
		int i = 0;//用于初始化mpu6050和hmc5883l
		SystemInit();//系统时钟配置
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//NVIC优先级组2
		
		Led_Configuration();//LED初始化，用于mpu6050和hmc5883l的初始化和Debug
		
		USART1_Config();//串口1的配置，DBUS RX；
		USART3_Configuration();//串口3的配置，用于调试；
		TIM2_Configuration();//用于计时
		delay_ms(500);    
		
		while(MPU6050_Initialization() == 0xff || HMC5883L_Initialization() == 0xff) 
    {
        i++;     //如果一次初始化没有成功，那就再来一次                     
        if(i>10) //如果初始化一直不成功，那就没希望了，进入死循环，绿色LED一直闪
        {
            while(1) 
            {
                LED_GREEN_TOGGLE();
                delay_ms(50);
                
            }
        }  
    } 
		
    MPU6050_Gyro_calibration();//MPU6050校准
    MPU6050_HMC5883L_Interrupt_Configuration(); //HMC和MPU的中断配置
		
		while(1)
		{		
			
			
			//串口测试例子
			//delay_ms(50);
			//sprintf(buffer1,"%d \r %d \r %f\n",Pitch.targetAngle,Pitch.thisAngle,MPU6050_Real_Data.Gyro_Y );
			//printf(buffer1);
			
		}
}
