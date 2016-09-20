#ifndef __MAIN_H__
#define __MAIN_H__

//�����
#include "stm32f4xx.h"
//�����
#include "usart1.h"
#include "usart3.h"
#include "timer.h"
//MPU6050 HMC5883L��
#include "mpu6050_hmc5883l_driver.h"
#include "mpu6050_hmc5883l_i2c.h"
#include "mpu6050_hmc5883l_interrupt.h"
#include "mpu6050_hmc5883l_process.h"
//�������Ժ���ʱ
#include "led.h"
#include "buzzer.h"
#include "delay.h"
//����������
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>

//���ú��� ȡ���ֵ�;���ֵ
#define abs(x) ((x)>0? (x):(-(x)))
#define maxs(a,b) (a>b? a:b)

//�������ڵ���printf
extern char buffer1[32];

#endif 
