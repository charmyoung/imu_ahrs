#include "main.h"

uint8_t last_s2;//s2����һ��key
uint8_t last_pressr;
int rub_flag;//s2-1λ����״̬
int shoot_flag;//s2-2λ����״̬

//rc��key����Ħ����BLDC����
void BLDC_control(uint8_t s2, uint8_t press_r)
{	
	//���̼�λ����
	
	//int key_G = KEY_PRESSED_OFFSET_SHIFT & v; if (key_G!=0) key_G=1;

	if (rub_flag == 1 && ( (s2==1 && last_s2!=s2) || (press_r == 1 && last_pressr!=press_r) ))
  {
    //pwm���Ƶ��2312 close��
			PWM1=1000;
			PWM2=1000;
		rub_flag=0;
  }
  else if (rub_flag == 0 && ( (s2==1 && last_s2!=s2) || (press_r == 1 && last_pressr!=press_r) ))
  {
    
		//pwm���Ƶ��2312 open��
    		PWM1=RUB_SPEED;
				PWM2=RUB_SPEED;
		rub_flag=1;
	}
	last_s2=s2;
	last_pressr=press_r;
}


//rc��mouse���Ʋ����������������
void Fire(uint8_t s2, uint8_t press_l)
{
	shoot_flag=0;

	if ((rub_flag == 1) && ( (s2==2) || (press_l == 1) ))
  {
		//pwm���Ʋ������ on��
    //�ʼҪ�ȳ�ʼ�����pwmΪ0��
		//l r �ֱ���Ƶ��ת������
		//lΪ��ת����pwm60/256��rΪ��ת��ֹ����195/256;
		//l r ͬʱ����ʱ�� ������ת���� pwm120 /256;
		//ң����s2 2λֻ��ʵ����ת���� pwm 60/256
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
