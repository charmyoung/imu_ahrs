#include "main.h"
int moveSpeed;//控制的速度RPM

RM3510_DATA RM3510_1={0,0,0,0,0,0,1};
RM3510_DATA RM3510_2={0,0,0,0,0,0,2};
RM3510_DATA RM3510_3={0,0,0,0,0,0,3};
RM3510_DATA RM3510_4={0,0,0,0,0,0,4};

RM35_DATA RM35_1;
RM35_DATA RM35_2;
RM35_DATA RM35_3;
RM35_DATA RM35_4;


//RM3510电调
void ChassisMotor_Velocity_Control(float vel1,float vel2,float vel3,float vel4)
{
	 RM3510_1.targetVelocity=vel1*19;
	 RM3510_2.targetVelocity=-vel2*19;
	 RM3510_3.targetVelocity=vel3*19;
	 RM3510_4.targetVelocity=-vel4*19;
	
	 RM3510_1.velocity_output = Velocity_Control_820R(RM3510_1.thisVelocity ,RM3510_1.targetVelocity);
	 RM3510_2.velocity_output = Velocity_Control_820R(RM3510_2.thisVelocity ,RM3510_2.targetVelocity);
	 RM3510_3.velocity_output = Velocity_Control_820R(RM3510_3.thisVelocity ,RM3510_3.targetVelocity);
	 RM3510_4.velocity_output = Velocity_Control_820R(RM3510_4.thisVelocity ,RM3510_4.targetVelocity);
	 
	 Cmd_ESC_820R( RM3510_1.velocity_output, RM3510_2.velocity_output, RM3510_3.velocity_output, RM3510_4.velocity_output);

}

void ChassisMotor_Position_Control(float pos1,float pos2,float pos3,float pos4)
{
	 RM3510_1.targetPosition=pos1;
	 RM3510_2.targetPosition=8191-pos2;
	 RM3510_3.targetPosition=pos3;
	 RM3510_4.targetPosition=8191-pos4;
	
	 RM3510_1.position_output = Position_Control_820R(RM3510_1.thisPosition, RM3510_1.targetPosition);
	 RM3510_1.velocity_output = Velocity_Control_820R(RM3510_1.thisVelocity ,RM3510_1.position_output);
	
	 RM3510_2.position_output = Position_Control_820R(RM3510_2.thisPosition, RM3510_2.targetPosition);
	 RM3510_2.velocity_output = Velocity_Control_820R(RM3510_2.thisVelocity ,RM3510_2.position_output);
	 
	 RM3510_3.position_output = Position_Control_820R(RM3510_3.thisPosition, RM3510_3.targetPosition);
	 RM3510_3.velocity_output = Velocity_Control_820R(RM3510_3.thisVelocity ,RM3510_3.position_output);
	 
	 RM3510_4.position_output = Position_Control_820R(RM3510_4.thisPosition, RM3510_4.targetPosition);
	 RM3510_4.velocity_output = Velocity_Control_820R(RM3510_4.thisVelocity ,RM3510_4.position_output);
	 
	 Cmd_ESC_820R( RM3510_1.velocity_output, RM3510_2.velocity_output, RM3510_3.velocity_output, RM3510_4.velocity_output);

}

//电机运动
void TransMove(int x,int y,int z,long temp_speed)
{
	long vf,vtr,vrr,vt,top_speed;
	top_speed=((long)maxs(abs(x),maxs(abs(y),abs(z)))*temp_speed)/660;
	vt=abs(x)+abs(y)+abs(z);
	vf=0;vtr=0;vrr=0;
	if (vt!=0)
	{
		vf=(y*top_speed)/vt;
		vtr=(x*top_speed)/vt;
		vrr=(z*top_speed)/vt;
	}
	//重置初始化和模式选择在main中写
	#ifdef RM35
	CAN_RoboModule_DRV_PWM_Velocity_Mode(0,1,5000,vf+vtr+vrr);	
	CAN_RoboModule_DRV_PWM_Velocity_Mode(0,2,5000,vf-vtr-vrr);
	CAN_RoboModule_DRV_PWM_Velocity_Mode(0,3,5000,vf-vtr+vrr);
	CAN_RoboModule_DRV_PWM_Velocity_Mode(0,4,5000,vf+vtr-vrr);
	#endif
	
	#ifdef RM3510
	ChassisMotor_Velocity_Control(vf+vtr+vrr,vf-vtr-vrr,vf-vtr+vrr,vf+vtr-vrr);
	#endif
}


//rc和key控制电机运动
void move_control(uint16_t ch0, uint16_t ch1, uint16_t ch2, uint8_t s1, uint16_t v)
{
	//键盘键位解析
				
				int key_SHIFT = KEY_PRESSED_OFFSET_SHIFT & v; 
				int key_CTRL = KEY_PRESSED_OFFSET_CTRL & v; 
				int key_W = KEY_PRESSED_OFFSET_W & v; 
        int key_A = KEY_PRESSED_OFFSET_A & v; 
        int key_S = KEY_PRESSED_OFFSET_S & v; 
        int key_D = KEY_PRESSED_OFFSET_D & v; 
        int key_Q = KEY_PRESSED_OFFSET_Q & v; 
        int key_E = KEY_PRESSED_OFFSET_E & v; 
				if (key_SHIFT!=0) key_SHIFT=1;
				if (key_CTRL!=0) key_CTRL=1;
				if (key_W!=0) key_W=1;
				if (key_A!=0) key_A=1;
				if (key_S!=0) key_S=1;
				if (key_D!=0) key_D=1;
				if (key_Q!=0) key_Q=1;
				if (key_E!=0) key_E=1;
				//未使用的键位
				if (KEY_PRESSED_OFFSET_R & v);//R pressed >>>> Reloading signal send
				if (KEY_PRESSED_OFFSET_F & v);//F pressed >>>> Laser on/off
				if (KEY_PRESSED_OFFSET_Z & v);//Z pressed
				if (KEY_PRESSED_OFFSET_X & v);//X pressed
				if (KEY_PRESSED_OFFSET_C & v);//C pressed
				if (KEY_PRESSED_OFFSET_V & v);//V pressed
				if (KEY_PRESSED_OFFSET_B & v);//B pressed
	//默认速度
	moveSpeed=80;
	if(s1==1){
	moveSpeed=160;}//可以考虑提高到180
	else if(s1==3){
	moveSpeed=90;}//提高到110
	else if(s1==2){
	moveSpeed=50;}//到60
	if(key_CTRL){
	moveSpeed=50;}
	if(key_SHIFT){
	moveSpeed=160;}
	//RC and KEY
	TransMove((ch0-1024)+(key_D-key_A)*660,(ch1-1024)+(key_W-key_S)*660,(ch2-1024)+(key_E-key_Q)*660,moveSpeed);
	
}



