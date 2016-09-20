#include "main.h"
MPU6050_RAW_DATA    MPU6050_Raw_Data; 
MPU6050_REAL_DATA   MPU6050_Real_Data;
HMC5883L_RAW_DATA		HMC5883L_Raw_Data; 
HMC5883L_REAL_DATA   HMC5883L_Real_Data;

//�趨�����Ǿ���
int gyroADC_X_offset=0,gyroADC_Y_offset=0,gyroADC_Z_offset=0;

//MPU6050 ��ʼ�����ɹ�����0  ʧ�ܷ��� 0xff
int MPU6050_Initialization(void)
{
    unsigned char temp_data = 0x00;
    IIC_GPIO_Init();  //��ʼ��IIC�ӿ�
    //HEAT_Configuration();
	
		
    
    if(IIC_ReadData(MPU6050_DEVICE_ADDRESS,WHO_AM_I ,&temp_data ,1)==0) //ȷ��IIC�����Ϲҽӵ��Ƿ���MPU6050
    {		
        if(temp_data != MPU6050_ID)
        {		
            printf("error MPU6050_Init_WrongID\r\n");
            return 0xff; //У��ʧ�ܣ�����0xff
        }
    }
    else
    {
        printf("error MPU6050_Init_DeviceLost\r\n");
        return 0xff; //��ȡʧ�� ����0xff
    }
    
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,PWR_MGMT_1,0x01) == 0xff)    //�������״̬
    {
        printf("error MPU6050_Init_PWR_MGMT_1\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,SMPLRT_DIV,0x07) == 0xff)//cyq��07 ����Ƶ��1khz
    {
        printf("error MPU6050_Init_SMPLRT_DIV\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,CONFIG,0x00) == 0xff)
    {
        printf("error MPU6050_Init_CONFIG\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,GYRO_CONFIG,0x08) == 0xff)
    {
        printf("error MPU6050_Init_GYRO_CONFIG\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,ACCEL_CONFIG,0x08) == 0xff)
    {
        printf("error MPU6050_Init_ACCEL_CONFIG\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_PIN_CFG,0x02) == 0xff)
    {
        printf("error MPU6050_Init_INT_PIN_CFG\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_ENABLE,0x01) == 0xff)
    {
        printf("error MPU6050_Init_INT_ENABLE\r\n");
        return 0xff;
    }
		
    
    return 0;
}

int HMC5883L_Initialization(void)
{
		unsigned char temp_data=0;
		 if(IIC_ReadData(HMC5883L_DEVICE_ADDRESS,WHO_AM_I ,&temp_data ,1)==0) //ȷ��IIC�����Ϲҽӵ��Ƿ���HMC5883l
    {		
        if(temp_data != 0x00)
        {		
            printf("error HMC5883L_Init_WrongID\r\n");
            return 0xff; //У��ʧ�ܣ�����0xff
        }
    }
    else
    {
        printf("error HMC5883L_Init_DeviceLost\r\n");
        return 0xff; //��ȡʧ�� ����0xff
    }
    
    if(IIC_WriteData(HMC5883L_DEVICE_ADDRESS,HMC5883L_RA_CONFIG_A,0x14) == 0xff)    //14:30hz--18:75Hz
   	{
        printf("error HMC5883L_Init_CONFIGA\r\n");
        return 0xff;
    }
		 if(IIC_WriteData(HMC5883L_DEVICE_ADDRESS,HMC5883L_RA_MODE,0x00) == 0xff)    //��������״̬
    {
        printf("error HMC5883L_Init_MODE\r\n");
        return 0xff;
    }
		

		return 0;

}
int HMC5883L_ReadData(void)
{
    u8 buf[6];
    
    if(IIC_ReadData(HMC5883L_DEVICE_ADDRESS,HMC5883L_DATA_START,buf,6) == 0xff)
    {
        printf("error HMC5883L_ReadData\r\n");
        return 0xff;
    }
    else
    {
        //��ȡ�Ĵ���ԭ������
           
        HMC5883L_Raw_Data.Mag_X = (buf[0]<<8 | buf[1]);
        HMC5883L_Raw_Data.Mag_Z = (buf[2]<<8 | buf[3]);
        HMC5883L_Raw_Data.Mag_Y = (buf[4]<<8 | buf[5]); 

				HMC5883L_Real_Data.Mag_X = HMC5883L_Raw_Data.Mag_X/1090;//��λGauss����������1090���㣬��hmc5883l�����п��޸�����ģʽ��
  			HMC5883L_Real_Data.Mag_Z = HMC5883L_Raw_Data.Mag_Z/1090;
        HMC5883L_Real_Data.Mag_Y = HMC5883L_Raw_Data.Mag_Y/1090; 
				
        //��ԭ������ת��Ϊʵ��ֵ�����㹫ʽ���Ĵ����������й�
       HMC5883L_Real_Data.Yaw_Angle= atan2(HMC5883L_Real_Data.Mag_Y,HMC5883L_Real_Data.Mag_X) * (180 / 3.14159265) + 180;
    } 
    
    return 0;
}
//MPU6050  ���ݶ�ȡ���ɹ�����0  ʧ�ܷ��� 0xff
int MPU6050_ReadData(void)
{
    u8 buf[14];
    
    if(IIC_ReadData(MPU6050_DEVICE_ADDRESS,MPU6050_DATA_START,buf,14) == 0xff)
    {
        printf("error MPU6050_ReadData\r\n");
        return 0xff;
    }
    else
    {
        //��ȡ�Ĵ���ԭ������
           
        MPU6050_Raw_Data.Accel_X = (buf[0]<<8 | buf[1]);
        MPU6050_Raw_Data.Accel_Y = (buf[2]<<8 | buf[3]);
        MPU6050_Raw_Data.Accel_Z = (buf[4]<<8 | buf[5]); 
        MPU6050_Raw_Data.Temp =    (buf[6]<<8 | buf[7]);  
        MPU6050_Raw_Data.Gyro_X = (buf[8]<<8 | buf[9]);
        MPU6050_Raw_Data.Gyro_Y = (buf[10]<<8 | buf[11]);
        MPU6050_Raw_Data.Gyro_Z = (buf[12]<<8 | buf[13]);

       
        //��ԭ������ת��Ϊʵ��ֵ�����㹫ʽ���Ĵ����������й�
        MPU6050_Real_Data.Accel_X = -(float)(MPU6050_Raw_Data.Accel_X)/8192.0; //��datasheet 30 of 47
        MPU6050_Real_Data.Accel_Y = -(float)(MPU6050_Raw_Data.Accel_Y)/8192.0; //��datasheet 30 of 47
        MPU6050_Real_Data.Accel_Z = (float)(MPU6050_Raw_Data.Accel_Z)/8192.0; //��datasheet 30 of 47
        MPU6050_Real_Data.Temp =   (float)(MPU6050_Raw_Data.Temp)/340.0+36.53;//��datasheet 31 of 47
        MPU6050_Real_Data.Gyro_X = -(float)(MPU6050_Raw_Data.Gyro_X - gyroADC_X_offset)/65.5;     //��datasheet 32 of 47
        MPU6050_Real_Data.Gyro_Y = -(float)(MPU6050_Raw_Data.Gyro_Y - gyroADC_Y_offset)/65.5;     //��datasheet 32 of 47
        MPU6050_Real_Data.Gyro_Z = (float)(MPU6050_Raw_Data.Gyro_Z - gyroADC_Z_offset)/65.5;     //��datasheet 32 of 47
    } 
    
    return 0;
}


void MPU6050_Gyro_calibration(void)
{
	u16 i;
	float x_temp=0,y_temp=0,z_temp=0;
	
	for(i=0; i<1000; i++)
	{
		MPU6050_ReadData();
		delay_ms(1);
		x_temp=x_temp+MPU6050_Raw_Data.Gyro_X;
		y_temp=y_temp+MPU6050_Raw_Data.Gyro_Y;
		z_temp=z_temp+MPU6050_Raw_Data.Gyro_Z;
	}			
	
	x_temp=x_temp/1000.00;
	y_temp=y_temp/1000.00;	
	z_temp=z_temp/1000.00;

	gyroADC_X_offset=(int)x_temp;
	gyroADC_Y_offset=(int)y_temp;
	gyroADC_Z_offset=(int)z_temp;
	
}
