#include "main.h"

uint8_t last_s2;//s2的上一次key
uint8_t last_pressr;
int rub_flag;//s2-1位开启状态
int shoot_flag;//s2-2位开启状态

//rc和key控制摩擦轮BLDC开启
void BLDC_control(uint8_t s2, uint8_t press_r)
{	
	//键盘键位解析
	
	//int key_G = KEY_PRESSED_OFFSET_SHIFT & v; if (key_G!=0) key_G=1;

	if (rub_flag == 1 && ( (s2==1 && last_s2!=s2) || (press_r == 1 && last_pressr!=press_r) ))
  {
    //pwm控制电调2312 close；
			PWM1=1000;
			PWM2=1000;
		rub_flag=0;
  }
  else if (rub_flag == 0 && ( (s2==1 && last_s2!=s2) || (press_r == 1 && last_pressr!=press_r) ))
  {
    
		//pwm控制电调2312 open；
    		PWM1=RUB_SPEED;
				PWM2=RUB_SPEED;
		rub_flag=1;
	}
	last_s2=s2;
	last_pressr=press_r;
}


//rc和mouse控制拨弹电机开启（开火）
void Fire(uint8_t s2, uint8_t press_l)
{
	shoot_flag=0;

	if ((rub_flag == 1) && ( (s2==2) || (press_l == 1) ))
  {
		//pwm控制拨弹电机 on；
    //最开始要先初始化电机pwm为0；
		//l r 分别控制电机转动方向，
		//l为正转拨弹pwm60/256，r为反转防止卡弹195/256;
		//l r 同时按下时候 加速正转拨弹 pwm120 /256;
		//遥控器s2 2位只能实现正转拨弹 pwm 60/256
    shoot_flag=1;
		
	}
  if (shoot_flag == 1)
	{
		PWM3=SHOOT_SPEED;
	}
	else if(shoot_flag == 0)
	{ PWM3=0;}
	
	last_s2=s2;
}
