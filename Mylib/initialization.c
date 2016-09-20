#include "main.h"

void  Initialization(void)
{
	
		int i = 0;//用于初始化mpu6050和hmc5883l
		SystemInit();//系统时钟配置
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//NVIC优先级组2
		
		Led_Configuration();//LED初始化，用于mpu6050和hmc5883l的初始化和Debug
		
		USART1_Config();//串口1的配置，DBUS RX；
		USART3_Configuration();//串口3的配置，用于调试；
		
		TIM2_Configuration();//用于计时
	
		CAN1_Configuration();//	Can1的配置，用于控制云台
		CAN2_Configuration();//Can2的配置，用于控制底盘	

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
		
		PWM_Configuration();//PWM配置，用于控制摩擦轮和拨弹电机
		delay_ms(500);
		
		#ifdef RM35
		CAN_RoboModule_DRV_Reset(0,0);//CAN重置
		delay_ms(1000);
		CAN_RoboModule_DRV_Mode_Choice(0,0,PWM_VELOCITY_MODE);//PWM速度模式
		delay_ms(1000);
		#endif
		
		//TIM6_Configuration();//定时器的配置，一般用于测试。
		//TIM6_Start();//定时器开启，一般用于测试
		//Cmd_Reset();云台校准，一般不用；
}
