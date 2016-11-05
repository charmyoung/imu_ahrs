#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"

//SDA    PC11
//SCL    PC12

//IO��������
#define SDA_IN()  {GPIOC->CRH&=0XFFFF0FFF;GPIOC->CRH|=8<<12;}
#define SDA_OUT() {GPIOC->CRH&=0XFFFF0FFF;GPIOC->CRH|=3<<12;}

//IO��������	 
#define IIC_SCL    PCout(12) //SCL
#define IIC_SDA    PCout(11) //SDA	 
#define READ_SDA   PCin(11)  //����SDA 

//0��ʾд
#define	I2C_Transmitter   0
//����ʾ��
#define	I2C_Receiver      1	

#define true 1
#define false 0 
#define bool  uint8_t


#define TRUE  0
#define FALSE -1




//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 

bool i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
bool i2cReadBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf);
uint8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data);
uint8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);


#endif
















