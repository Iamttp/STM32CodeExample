#include "mpu6050.h"
#include "delay.h"


/**********************************************
*i2c����   i2c_gpio_init();i2c_start();i2c_stop();
*		   i2c_wait_ack();i2c_ack();i2c_waitack();
*		   IIC_Send_Byte();IIC_Read_Byte();
**********************************************/
//IO��������
//IO��������
#define SDA_IN()  {GPIOB->CRH&=0XFFFFFFF0;GPIOB->CRH|=8<<0;}
#define SDA_OUT() {GPIOB->CRH&=0XFFFFFFF0;GPIOB->CRH|=3<<0;}

//IO��������	 
#define SCL    PBout(9) 		//SCL
#define SDA_w    PBout(8) 		//SDA	 
#define SDA_r  PBin(8) 		//����SDA 

void i2c_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_TypeDefstruct;
	 GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ��PB�˿�ʱ��
	GPIO_TypeDefstruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_TypeDefstruct.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9;
	GPIO_TypeDefstruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_TypeDefstruct);
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);//��ʹ������IO PORTCʱ�� 
		
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_11;	 // �˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
  GPIO_Init(GPIOC, &GPIO_InitStructure);					 //�����趨������ʼ��GPIO 
	
  GPIO_SetBits(GPIOC,GPIO_Pin_12|GPIO_Pin_11);						 //PB10,PB11 �����	
 
	SCL=1;
	SDA_w=1;
}

void i2c_start(void)
{
	SDA_OUT();	
	SDA_w=1;
	SCL=1;									//��������������������������������������������									////
	delay_us(2);
	SDA_w=0;
	delay_us(2);
	SCL=0;
}

void i2c_stop(void)
{
	SDA_OUT();
	SCL=0;
	SDA_w=0;
	delay_us(2);
	SCL=1;
	SDA_w=1;
	delay_us(2);
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 i2c_wait_ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	SDA_w=1;delay_us(2);	   
	SCL=1;delay_us(2);	 
	while(SDA_r)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			i2c_stop();
			return 1;
		}
	}
	SCL=0;//ʱ�����0 	   
	return 0;  
}
//���豸�յ����ݺ���Զ�����ACK���ж��豸�治���ڿ��Ը���ACK�ź�
//����ACKӦ��
void i2c_ack(void)
{
	SCL=0;
	SDA_OUT();
	SDA_w=0;
	delay_us(2);
	SCL=1;
	delay_us(2);
	SCL=0;
}
//������ACKӦ��		    
void i2c_nack(void)
{
	SCL=0;
	SDA_OUT();
	SDA_w=1;
	delay_us(2);
	SCL=1;
	delay_us(2);
	SCL=0;
}	
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        SDA_w=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		SCL=1;
		delay_us(2); 
		SCL=0;	
		delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        SCL=0; 
        delay_us(2);
		SCL=1;
        receive<<=1;
        if(SDA_r)receive++;   
		delay_us(2); 
    }					 
    if (!ack)
        i2c_nack();//����nACK
    else
        i2c_ack(); //����ACK   
    return receive;
}

/****************************************************
*mpu6050����  MPU_Write_Byte();MPU_Read_Byte();
*			  MPU_Init();
*			  MPU_Set_LPF();MPU_Set_Rate();
****************************************************/
//IICдһ���ֽ� 
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������

//ʱ������� 
//1����ʼ�źţ� 
//2��д���豸��ַ����ַ���λR/W=0����ʾΪд���������豸�յ����ݺ���Զ�����ACK���ж��豸�治���ڿ��Ը���ACK�źţ� 
//3��д�洢��ַ�߰�λ��д���ַ����豸�᷵��ACK����(�е��豸��10bit�ĵ�ַ��24C64��8bit��ַ) 
//4��д�洢��ַ�Ͱ�λ��д���ַ����豸�᷵��ACK�� 
//5��д8bit����(1Byte) 
//6��дֹͣλ
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
    i2c_start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����	
	if(i2c_wait_ack())	//�ȴ�Ӧ��
	{
		i2c_stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    i2c_wait_ack();		//�ȴ�Ӧ�� 
	IIC_Send_Byte(data);//��������
	if(i2c_wait_ack())	//�ȴ�ACK
	{
		i2c_stop();	 
		return 1;		 
	}		 
    i2c_stop();	 
	return 0;
}
//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������

//ʱ������� 
//1����ʼ�źţ� 
//2��д���豸��ַ����ַ���λR/W=1����ʾΪ�����������豸�յ����ݺ���Զ�����ACK���ж��豸�治���ڿ��Ը���ACK�źţ� 
//3�����洢��ַ�߰�λ��д���ַ����豸�᷵��ACK����(�е��豸��10bit�ĵ�ַ��24C64��8bit��ַ) 
//4�����洢��ַ�Ͱ�λ��д���ַ����豸�᷵��ACK�� 
//5��дֹͣλ 
//6��д��ʼ�ź� 
//7��д���豸��ַ 
//8����ȡ���ݣ������ߴ��豸��β��û�ACK 
//9��дֹͣλ			??????
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    i2c_start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����	
	i2c_wait_ack();		//�ȴ�Ӧ�� 
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    i2c_wait_ack();		//�ȴ�Ӧ��
    i2c_start();
	IIC_Send_Byte((MPU_ADDR<<1)|1);//����������ַ+������	
    i2c_wait_ack();		//�ȴ�Ӧ�� 
	res=IIC_Read_Byte(0);//��ȡ����,����nACK 
    i2c_stop();			//����һ��ֹͣ���� 
	return res;		
}

//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}

//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}



//��ʼ��MPU6050
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Init(void)
{ 
	u8 res; 
	i2c_gpio_init();//��ʼ��IIC����
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//��λMPU6050
    delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//����MPU6050 
	MPU_Write_Byte(MPU_GYRO_CFG_REG,3<<3);					//�����Ǵ�����,��2000dps
	MPU_Write_Byte(MPU_ACCEL_CFG_REG,0<<3);					//���ٶȴ�����,��2g
	MPU_Set_Rate(50);						//���ò�����50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//�ر������ж�
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG); 
	if(res==MPU_ADDR)//����ID��ȷ
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
		MPU_Set_Rate(50);						//���ò�����Ϊ50Hz
 	}else return 1;
	return 0;
}

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	i2c_start(); 
	IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(i2c_wait_ack())	//�ȴ�Ӧ��
	{
		i2c_stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    i2c_wait_ack();		//�ȴ�Ӧ��
    i2c_start();
	IIC_Send_Byte((addr<<1)|1);//����������ַ+������	
    i2c_wait_ack();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);//������,����nACK 
		else *buf=IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++; 
	}    
    i2c_stop();	//����һ��ֹͣ���� 
	return 0;	
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}

