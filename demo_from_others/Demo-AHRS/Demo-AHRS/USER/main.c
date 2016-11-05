/******************************************************************************
** 工程：   	STM32-MPU9250驱动
** 作者：	王小鹏
** 修改日志：
2016-5-11
1.使用DMP姿态解算，YAW偏移过大

2016-5-12
1.增加AHRS惯性导航IMU算法
********************************************************************************/


#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "lcd.h"
#include "key.h"  
#include "24cxx.h" 
#include "myiic.h"
#include "nvic.h"
#include "mpu9250.h"
#include "imu.h"
#include "math.h"

SENSOR_DATA Gyrobuf;//陀螺仪
SENSOR_DATA Accbuf;//加速度
SENSOR_DATA Magbuf;//磁力计

IMU_DATA GyroFinal;
IMU_DATA AccFinal;
IMU_DATA MagFinal;

float Pitch;
float Roll;
float Yaw;



int main(void)
{ 
	int count;
	delay_init();	    				//延时函数初始化	  
	uart_init(115200);	 				//串口初始化为9600
	LED_Init();		  					//初始化与LED连接的硬件接口 	
	IIC_Init();							//I2C初始化//SDA-PC11   SCL-PC12	
	Init_MPU9250();						//初始化MPU9250 
	Gyro_Correct();//陀螺仪校正
	Acc_Correct();//加速度计校正
	//Gyro_Correct();//电子罗盘校正
	printf("start runing...\n");
	while(1)
	{
		count++;
		AHRS_Dataprepare();//准备数据
		AHRSupdate(GyroFinal.X,GyroFinal.Y,GyroFinal.Z,
					AccFinal.X,AccFinal.Y,AccFinal.Z,
					MagFinal.X,MagFinal.Y,MagFinal.Z);
		if(count % 20 == 0)
		{
			/*printf("GyroFinal.X = %.2f\n",GyroFinal.X);
			printf("GyroFinal.Y = %.2f\n",GyroFinal.Y);
			printf("GyroFinal.Z = %.2f\n",GyroFinal.Z);
			printf("AccFinal.X = %.2f\n",AccFinal.X);
			printf("AccFinal.Y = %.2f\n",AccFinal.Y);
			printf("AccFinal.Z = %.2f\n",AccFinal.Z);
			printf("MagFinal.X = %.2f\n",MagFinal.X);
			printf("MagFinal.Y = %.2f\n",MagFinal.Y);
			printf("MagFinal.Z = %.2f\n",MagFinal.Z);*/
			
			
			
			printf("Pitch:%.2f  \r",Pitch);
			printf("Roll:%.2f   \r",Roll);
			printf("Yaw:%.2f    \r",Yaw);
			printf("\r\n");
		}
		delay_ms(10);
		if(count > 100000)
		{
			count = 0;
		}
	
	}
}
