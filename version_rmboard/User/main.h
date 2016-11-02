#ifndef __MAIN_H__
#define __MAIN_H__

//引入库
#include "stm32f4xx.h"
//引入库
#include "usart1.h"
#include "usart3.h"
#include "timer.h"
//MPU6050 HMC5883L库
#include "mpu6050_hmc5883l_driver.h"
#include "mpu6050_hmc5883l_i2c.h"
#include "mpu6050_hmc5883l_interrupt.h"
#include "mpu6050_hmc5883l_process.h"
//辅助调试和延时
#include "led.h"
#include "buzzer.h"
#include "delay.h"
//其他基本库
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>

//常用函数 取最大值和绝对值
#define abs(x) ((x)>0? (x):(-(x)))
#define maxs(a,b) (a>b? a:b)

//用作串口调试printf
extern char buffer1[32];

#endif 
