/******************************************************************************
** ���̣�   	STM32-MPU9250����
** ���ߣ�	��С��
** �޸���־��
2016-5-11
1.ʹ��DMP��̬���㣬YAWƫ�ƹ���

2016-5-12
1.����AHRS���Ե���IMU�㷨
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

SENSOR_DATA Gyrobuf;//������
SENSOR_DATA Accbuf;//���ٶ�
SENSOR_DATA Magbuf;//������

IMU_DATA GyroFinal;
IMU_DATA AccFinal;
IMU_DATA MagFinal;

float Pitch;
float Roll;
float Yaw;



int main(void)
{ 
	int count;
	delay_init();	    				//��ʱ������ʼ��	  
	uart_init(115200);	 				//���ڳ�ʼ��Ϊ9600
	LED_Init();		  					//��ʼ����LED���ӵ�Ӳ���ӿ� 	
	IIC_Init();							//I2C��ʼ��//SDA-PC11   SCL-PC12	
	Init_MPU9250();						//��ʼ��MPU9250 
	Gyro_Correct();//������У��
	Acc_Correct();//���ٶȼ�У��
	//Gyro_Correct();//��������У��
	printf("start runing...\n");
	while(1)
	{
		count++;
		AHRS_Dataprepare();//׼������
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
