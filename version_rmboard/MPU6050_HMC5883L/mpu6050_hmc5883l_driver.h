#ifndef __MPU6050_DRIVER_H__
#define __MPU6050_DRIVER_H__

typedef struct __MPU6050_RAW_Data__
{
    short Accel_X;  //�Ĵ���ԭ��X����ٶȱ�ʾֵ
    short Accel_Y;  //�Ĵ���ԭ��Y����ٶȱ�ʾֵ
    short Accel_Z;  //�Ĵ���ԭ��Z����ٶȱ�ʾֵ
    short Temp;     //�Ĵ���ԭ���¶ȱ�ʾֵ
    short Gyro_X;   //�Ĵ���ԭ��X�������Ǳ�ʾֵ
    short Gyro_Y;   //�Ĵ���ԭ��Y�������Ǳ�ʾֵ
    short Gyro_Z;   //�Ĵ���ԭ��Z�������Ǳ�ʾֵ
}MPU6050_RAW_DATA;

typedef struct __MPU6050_REAL_Data__
{
    float Accel_X;  //ת����ʵ�ʵ�X����ٶȣ�
    float Accel_Y;  //ת����ʵ�ʵ�Y����ٶȣ�
    float Accel_Z;  //ת����ʵ�ʵ�Z����ٶȣ�
    float Temp;     //ת����ʵ�ʵ��¶ȣ���λΪ���϶�
    float Gyro_X;   //ת����ʵ�ʵ�X��Ǽ��ٶȣ�
    float Gyro_Y;   //ת����ʵ�ʵ�Y��Ǽ��ٶȣ�
    float Gyro_Z;   //ת����ʵ�ʵ�Z��Ǽ��ٶ�
}MPU6050_REAL_DATA;

typedef struct __HMC5883L_RAW_Data__
{
    short Mag_X;  //�Ĵ���ԭ��X������Ʊ�ʾֵ
    short Mag_Y;  //�Ĵ���ԭ��Y������Ʊ�ʾֵ
    short Mag_Z;  //�Ĵ���ԭ��Z������Ʊ�ʾֵ

}HMC5883L_RAW_DATA;

typedef struct __HMC5883L_REAL_Data__
{
    float Mag_X;  //ת����ʵ�ʵ�X����ٶȣ�
    float Mag_Y;  //ת����ʵ�ʵ�Y����ٶȣ�
    float Mag_Z;  //ת����ʵ�ʵ�Z����ٶȣ�
    float Yaw_Angle;   //ת����ʵ�ʵ�X��Ǽ��ٶȣ�
}HMC5883L_REAL_DATA;

extern MPU6050_RAW_DATA    MPU6050_Raw_Data; 
extern MPU6050_REAL_DATA   MPU6050_Real_Data;
extern HMC5883L_RAW_DATA   HMC5883L_Raw_Data;
extern HMC5883L_REAL_DATA   HMC5883L_Real_Data;

//����MPU6050�ڲ���ַ
#define	SMPLRT_DIV			0x19	//�����ǲ����� ����ֵ 0X07 125Hz
#define	CONFIG					0x1A	//��ͨ�˲�Ƶ�� ����ֵ 0x00 
#define	GYRO_CONFIG			0x1B	//�������Լ켰������Χ                 ����ֵ 0x18 ���Լ� 2000deg/s
#define	ACCEL_CONFIG		0x1C	//���ٶȼ��Լ켰������Χ����ͨ�˲�Ƶ�� ����ֵ 0x01 ���Լ� 2G 5Hz

#define INT_PIN_CFG     0x37
#define INT_ENABLE      0x38
#define INT_STATUS      0x3A    //ֻ��

#define	ACCEL_XOUT_H		0x3B
#define	ACCEL_XOUT_L		0x3C

#define	ACCEL_YOUT_H		0x3D
#define	ACCEL_YOUT_L		0x3E

#define	ACCEL_ZOUT_H		0x3F
#define	ACCEL_ZOUT_L		0x40
	
#define	TEMP_OUT_H			0x41
#define	TEMP_OUT_L			0x42

#define	GYRO_XOUT_H			0x43
#define	GYRO_XOUT_L			0x44	

#define	GYRO_YOUT_H			0x45
#define	GYRO_YOUT_L			0x46

#define	GYRO_ZOUT_H			0x47
#define	GYRO_ZOUT_L			0x48

#define	PWR_MGMT_1			0x6B	//��Դ���� ����ֵ 0x00 ��������
#define	WHO_AM_I				0x75	//ֻ��  Ĭ�϶���Ӧ���� MPU6050_ID = 0x68


//����MPU6050��Ϣ
#define MPU6050_ID              0x68
#define MPU6050_DEVICE_ADDRESS  0xD0
#define MPU6050_DATA_START      ACCEL_XOUT_H   //�������ݴ�ŵ�ַ�������ģ�����һ������

//����HMC5883L�ڲ���ַ
#define HMC5883L_RA_CONFIG_A        0x00
#define HMC5883L_RA_CONFIG_B        0x01
#define HMC5883L_RA_MODE            0x02
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAX_L         0x04
#define HMC5883L_RA_DATAY_H         0x05
#define HMC5883L_RA_DATAY_L         0x06
#define HMC5883L_RA_DATAZ_H         0x07
#define HMC5883L_RA_DATAZ_L         0x08
#define HMC5883L_RA_STATUS          0x09
#define HMC5883L_RA_ID_A            0x0A
#define HMC5883L_RA_ID_B            0x0B
#define HMC5883L_RA_ID_C            0x0C

//����HMC5883L��Ϣ
#define HMC5883L_DEVICE_ADDRESS 0x3c
#define HMC5883L_ID 					  0x00
#define HMC5883L_DATA_START     HMC5883L_RA_DATAX_H

//MPU6050��ʼ������ȡ���ݡ�У��
int MPU6050_Initialization(void);
int MPU6050_ReadData(void);
void MPU6050_Gyro_calibration(void);

//HMC5883L��ʼ������ȡ����
int HMC5883L_Initialization(void);
int HMC5883L_ReadData(void);

#endif
