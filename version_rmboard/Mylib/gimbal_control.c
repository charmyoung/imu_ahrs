#include "main.h"

//初始化云台角度
M6623 Yaw = {YAW_LEFT,YAW_RIGHT,YAW_MID,0,0,0,0,YAW_MID,0,0};
M6623 Pitch = {PITCH_DOWN,PITCH_UP,PITCH_MID,0,0,0,0,PITCH_MID,0,0};

/*********************************************************************
Name：          void Gimbal_Control(void)  

Description：  云台控制程序
               向上运动电流为正值
*********************************************************************/
void Gimbal_Control(void)  
{
	//外环PID控制
	//计算位置闭环输出量
	Yaw.position_output = Position_Control_205(Yaw.thisAngle,Yaw.targetAngle);
	//内环PID控制
  Yaw.velocity_output = Velocity_Control_205(-MPU6050_Real_Data.Gyro_Z ,Yaw.position_output);

	//计算位置闭环输出量
	Pitch.position_output = Position_Control_206(Pitch.thisAngle,Pitch.targetAngle);
	//内环PID控制
  Pitch.velocity_output = Velocity_Control_206(-MPU6050_Real_Data.Gyro_Y ,Pitch.position_output);
	Cmd_ESC(Yaw.velocity_output,Pitch.velocity_output);
	
}

/*********************************************************************
Name：         void Trigger_Control(int16_t x, int16_t y, uint16_t ch3)

Description：  云台远程控制程序（遥控器和键盘）            
*********************************************************************/

void Trigger_Control(int16_t x, int16_t y, uint16_t ch3)
{
				static int y_flag;
				//if (x>2) Yaw.targetAngle += 30;
        //if (x<-2) Yaw.targetAngle += -30;		
				//遥控器2通道测试yaw轴云台
				//if (x==0 && dbus.rc.ch2 != 0) Yaw.targetAngle=(Yaw.minAngle+(((float)dbus.rc.ch2-364.)/(1684.-364.))*(float)(Yaw.maxAngle-Yaw.minAngle))/50*50;
				
				if (Yaw.targetAngle < Yaw.minAngle){Yaw.targetAngle=Yaw.minAngle;}
				if (Yaw.targetAngle > Yaw.maxAngle){Yaw.targetAngle=Yaw.maxAngle;}

			
				if (y>2) {Pitch.targetAngle += -30;y_flag=1;}
        if (y<-2) {Pitch.targetAngle += 30;y_flag=1;}
				if (y_flag != 1 && ch3 != 0) 
				{
					if(ch3>=1024) Pitch.targetAngle=(Pitch.defualtAngle+(((float)ch3-1024.)/(1684.-1024.))*(float)(Pitch.maxAngle-Pitch.defualtAngle))/50*50;
					if(ch3<1024) Pitch.targetAngle=(Pitch.defualtAngle+(((float)ch3-1024.)/(364.-1024.))*(float)(Pitch.minAngle-Pitch.defualtAngle))/50*50;				
				}
				if (Pitch.targetAngle < Pitch.minAngle){Pitch.targetAngle=Pitch.minAngle;}
				if (Pitch.targetAngle > Pitch.maxAngle){Pitch.targetAngle=Pitch.maxAngle;}		
			
}
