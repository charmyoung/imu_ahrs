#include "main.h"

ACCEL_AVERAGE_DATA   Accel_Raw_Average_Data; //���ٶȵ�ƽ��ֵ
GYRO_RADIAN_DATA     Gyro_Radian_Data;//����������
AHRS_DATA        AHRS_Data = {0.,0.,0., 0.,0.,0., 1,0,0,0, 0.,0.,0.};

volatile uint32_t lastUpdate, now; // �������ڼ��� ��λ us

void MPU6050_Data_Filter(void)  //����FIFO��˼����ƽ��ֵ
	{
    unsigned int i=0;
    static unsigned int first_flag = 0;
    static unsigned int filter_cnt = 0;    //������ٶȼ��˲��Ĵ���
    
    long temp_accel_x = 0; //������ż��ٶȼ�X��ԭ�����ݵ��ۼӺ�
    long temp_accel_y = 0; //������ż��ٶȼ�Y��ԭ�����ݵ��ۼӺ�
    long temp_accel_z = 0; //������ż��ٶȼ�Z��ԭ�����ݵ��ۼӺ�
    
    static short accel_x_buffer[10] = {0}; //������ż��ٶȼ�X�����10�����ݵ�����
    static short accel_y_buffer[10] = {0}; //������ż��ٶȼ�Y�����10�����ݵ�����
    static short accel_z_buffer[10] = {0}; //������ż��ٶȼ�Z�����10�����ݵ�����
    
    if(first_flag == 0) //�����һ�ν����ú��������������ƽ����������г�ʼ��
    {
        first_flag = 1; //�Ժ��ٽ���
        for(i=0;i<10;i++)
        {
            accel_x_buffer[i] = MPU6050_Raw_Data.Accel_X;
            accel_y_buffer[i] = MPU6050_Raw_Data.Accel_Y;
            accel_z_buffer[i] = MPU6050_Raw_Data.Accel_Z;
        }
    }
    else  //������ǵ�һ����
    {
        accel_x_buffer[filter_cnt] = MPU6050_Raw_Data.Accel_X;
        accel_y_buffer[filter_cnt] = MPU6050_Raw_Data.Accel_Y;
        accel_z_buffer[filter_cnt] = MPU6050_Raw_Data.Accel_Z;   
        
        filter_cnt ++;
        if(filter_cnt == 10)
        {
            filter_cnt = 0;
        }        
    }
    
    for(i=0;i<10;i++)
    {
        temp_accel_x += accel_x_buffer[i];
        temp_accel_y += accel_y_buffer[i];
        temp_accel_z += accel_z_buffer[i];
    }
    
    Accel_Raw_Average_Data.X = (float)temp_accel_x / 10.0;
    Accel_Raw_Average_Data.Y = (float)temp_accel_y / 10.0;
    Accel_Raw_Average_Data.Z = (float)temp_accel_z / 10.0;
    
    Gyro_Radian_Data.X = (float)(MPU6050_Real_Data.Gyro_X  * (3.14159265/180.0));
    Gyro_Radian_Data.Y = (float)(MPU6050_Real_Data.Gyro_Y  * (3.14159265/180.0));
    Gyro_Radian_Data.Z = (float)(MPU6050_Real_Data.Gyro_Z  * (3.14159265/180.0));
}


/**************************Fast inverse square-rootʵ�ֺ���********************************************
*����ԭ��:	   float invSqrt(float x)
*��������:	   ���ټ��� 1/Sqrt(x) 	
��������� Ҫ�����ֵ
��������� ���
*******************************************************************************/
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void AHRS_Calculate( float gyro_x,
                              float gyro_y,
                              float gyro_z,
                              float accel_x,
                              float accel_y,
                              float accel_z,
															float mag_x,
															float mag_y,
															float mag_z)
{		 
		unsigned int i=0;
		static unsigned int initial_count = 0;//��ʼ����Ԫ��count
		const unsigned int countnum=1000;
				
		static float ini_ax[countnum];
		static float ini_ay[countnum];
		static float ini_az[countnum];
		static float ini_mx[countnum];
		static float ini_my[countnum];
		static float ini_mz[countnum];
		
		float temp_ini_ax;
		float temp_ini_ay;
		float temp_ini_az;
		float temp_ini_mx;
		float temp_ini_my;
		float temp_ini_mz;
		float ini_theta;
		float ini_phi;
		float ini_psi;
	
		static float q0 = 1;
    static float q1 = 0;
    static float q2 = 0;
    static float q3 = 0;
		
    static float exInt = 0;
    static float eyInt = 0;
    static float ezInt = 0;
    
    const float kp_const = 0.3; //
		float kp;
    const float ki = 0.00; //0.0;
		const float gravity = 9.81;
		unsigned int stationary = 0;
		
		float halfT; //�������ڵ�һ��ֵ
    
    float norm; //������ģ
    
		float hx,hy,hz;
		float bx,bz;
		float wx,wy,wz;
	
    float vx,vy,vz;
		
		float ex,ey,ez;

    float ax,ay,az; //���ٶ�������ģ�ı�ֵ 
    float gx,gy,gz; //������
		float mx,my,mz;//������

    static float pre_ax = 0;
    static float pre_ay = 0;
    static float pre_az = 0;
    
		float accel_xn;
		float accel_yn;
		float accel_zn;
		
		static float vel_xn;
		static float vel_yn;
		static float vel_zn;
		
		static float pos_xn;
		static float pos_yn;
		static float pos_zn;
		//ʱ��ȷ��
				now = Get_Time_Micros();  //��ȡʱ�� ��λ��us   
				if(now<lastUpdate)
				{
				//halfT =  ((float)(now + (0xffffffff- lastUpdate)) / 2000000.0f);   //  uint 0.5s
				}
				else	
				{
						halfT =  ((float)(now - lastUpdate) / 2000000.0f);
				}
				lastUpdate = now;	//����ʱ��
		
		//���ٶ��˲�(һ���ͺ��˲�)
		
    accel_x = accel_x *0.02 + pre_ax * 0.98;//cyq:0.02
    pre_ax = accel_x;
    
    accel_y = accel_y *0.02 + pre_ay * 0.98;
    pre_ay = accel_y;

    accel_z = accel_z *0.02 + pre_az * 0.98;
    pre_az = accel_z;    
    
		//��һ��ʵ�ʼ��ٶ�����
    stationary = abs(sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z)-gravity)<0.1;
		
		norm = invSqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    ax = accel_x * norm;
    ay = accel_y * norm;
    az = accel_z * norm;
		
		//��һ��ʵ�ʴ���������
    norm = invSqrt(mag_x * mag_x + mag_y * mag_y + mag_z * mag_z);
    mx = mag_x * norm;
    my = mag_y * norm;
    mz = mag_z * norm;
		

		if(initial_count <countnum)
		{
			ini_ax[initial_count]=ax;
			ini_ay[initial_count]=ay;
			ini_az[initial_count]=az;
			
			ini_mx[initial_count]=mx;
			ini_my[initial_count]=my;
			ini_mz[initial_count]=mz;
		}
		else if(initial_count == countnum)
		{   
			for(i=0;i<1000;i++)
			{		
        temp_ini_ax += ini_ax[i];
        temp_ini_ay += ini_ay[i];
        temp_ini_az += ini_az[i];
				
				temp_ini_mx += ini_mx[i];
        temp_ini_my += ini_my[i];
        temp_ini_mz += ini_mz[i];
			}
			ini_theta=-atan(temp_ini_ax/countnum);
			ini_phi=atan(temp_ini_ay/temp_ini_az);
			ini_psi=atan(mag_y/mag_x);
			
			q0 = cos(ini_phi/2)*cos(ini_theta/2)*cos(ini_psi/2)+sin(ini_phi/2)*sin(ini_theta/2)*sin(ini_psi/2);
			q1 = sin(ini_phi/2)*cos(ini_theta/2)*cos(ini_psi/2)-cos(ini_phi/2)*sin(ini_theta/2)*sin(ini_psi/2);
			q2 = cos(ini_phi/2)*sin(ini_theta/2)*cos(ini_psi/2)+sin(ini_phi/2)*cos(ini_theta/2)*sin(ini_psi/2);
			q3 = cos(ini_phi/2)*cos(ini_theta/2)*sin(ini_psi/2)+sin(ini_phi/2)*sin(ini_theta/2)*cos(ini_psi/2);
			
			
			norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
			q0 = q0 * norm;
			q1 = q1 * norm;
			q2 = q2 * norm;
			q3 = q3 * norm;
			LED_GREEN_OFF();
		}
		else{
				
				
				
				//����ο��µĴ�����
				hx = 2.0f*mx*(0.5f - q2*q2 - q3*q3) + 2.0f*my*(q1*q2 - q0*q3) + 2.0f*mz*(q1*q3 + q0*q2);
				hy = 2.0f*mx*(q1*q2 + q0*q3) + 2.0f*my*(0.5f - q1*q1 - q3*q3) + 2.0f*mz*(q2*q3 - q0*q1);
				hz = 2.0f*mx*(q1*q3 - q0*q2) + 2.0f*my*(q2*q3 + q0*q1) + 2.0f*mz*(0.5f - q1*q1 - q2*q2);         
				bx = sqrt((hx*hx) + (hy*hy));
				bz = hz;

				wx = 2.0f*bx*(0.5f - q2*q2 - q3*q3) + 2.0f*bz*(q1*q3 - q0*q2);
				wy = 2.0f*bx*(q1*q2 - q0*q3) + 2.0f*bz*(q0*q1 + q2*q3);
				wz = 2.0f*bx*(q0*q2 + q1*q3) + 2.0f*bz*(0.5f - q1*q1 - q2*q2); 
				
				
				//����ο��µļ��ٶ�ֵ
				vx = 2 * (q1*q3 - q0*q2);
				vy = 2 * (q0*q1 + q2*q3);
				vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
				
				
				//��������ֵ
				ex = (ay*vz - az*vy) + (my*wz - mz*wy);
				ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
				ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
				
				if(stationary)
				{
					kp = 0;
				}
				else{kp = kp_const;}
				
				if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
				{   
					exInt += ki*ex;
					eyInt += ki*ey;
					ezInt += ki*ez;
				
					gx = gyro_x + kp*ex + exInt;
					gy = gyro_y + kp*ey + eyInt;
					gz = gyro_z + kp*ez + ezInt;
				}
				
				q0 += (      - q1*gx - q2*gy - q3*gz)*halfT;
				q1 += (q0*gx +         q2*gz - q3*gy)*halfT;
				q2 += (q0*gy - q1*gz +         q3*gx)*halfT;
				q3 += (q0*gz + q1*gy - q2*gx        )*halfT;
				
				norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
				q0 = q0 * norm;
				q1 = q1 * norm;
				q2 = q2 * norm;
				q3 = q3 * norm;
				
				accel_xn = accel_x*(q0*q0+q1*q1-q2*q2-q3*q3) + accel_y*(2*q1*q2-2*q0*q3)         + accel_z*(2*q0*q2+2*q1*q3);
				accel_yn = accel_x*(2*q1*q2+2*q0*q3)				 + accel_y*(q0*q0-q1*q1+q2*q2-q3*q3);
				accel_zn = accel_x*(2*q1*q3-2*q0*q2)                                             + accel_z*(q0*q0-q1*q1-q2*q2+q3*q3);
				
				vel_xn += accel_xn * 2* halfT;
				vel_yn += accel_yn * 2* halfT;
				vel_zn += (accel_zn-gravity) * 2* halfT;
				
				if(stationary)
				{
					vel_xn = 0;
					vel_yn = 0;
					vel_zn = 0;
				}
				
				pos_xn += vel_xn * 2 * halfT;
				pos_yn += vel_yn * 2 * halfT;
				pos_zn += vel_zn * 2 * halfT;
				
				AHRS_Data.Acce_Nav_x=accel_xn;
				AHRS_Data.Acce_Nav_y=accel_yn;
				AHRS_Data.Acce_Nav_z=accel_zn;
				
				AHRS_Data.Vel_Nav_x=vel_xn;
				AHRS_Data.Vel_Nav_y=vel_yn;
				AHRS_Data.Vel_Nav_z=vel_zn;
				
				AHRS_Data.Pos_Nav_x=pos_xn;
				AHRS_Data.Pos_Nav_y=pos_yn;
				AHRS_Data.Pos_Nav_z=pos_zn;
				
			}
			//��Ԫ��ת��Ϊŷ����
				AHRS_Data.q0=q0;
				AHRS_Data.q1=q1;
				AHRS_Data.q2=q2;
				AHRS_Data.q3=q3;
				
				AHRS_Data.EulerAngle_Roll = asin(-2 * q1 * q3 + 2 * q0* q2) * (180.0/3.14159265); 
				AHRS_Data.EulerAngle_Pitch  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1) * (180.0/3.14159265); 
				AHRS_Data.EulerAngle_Yaw= atan2( 2 * q1 * q2 + 2 * q0 * q3,1.0 - 2.0 * ( q2 * q2 + q3 * q2 ) ) * (180.0/3.14159265);//��׼
			initial_count++;
}

