#ifndef __MPU_H
#define __MPU_H	 
#include "sys.h"

#define	SMPLRT_DIV		0x19	//??????,???:0x07(125Hz)
#define	CONFIG			0x1A	//??????,???:0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//??????????,???:0x18(???,2000deg/s)
#define	ACCEL_CONFIG	0x1C	//?????????????????,???:0x01(???,2G,5Hz)

#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

		
#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08


#define	PWR_MGMT_1		0x6B	//????,???:0x00(????)
#define	WHO_AM_I		  0x75	//IIC?????(????0x68,??)


//****************************

#define	GYRO_ADDRESS   0xD0	  //????
#define MAG_ADDRESS    0x18   //????
#define ACCEL_ADDRESS  0xD0 

extern short T_X,T_Y,T_Z,T_T;		 //X,Y,Z?,??
extern unsigned char TX_DATA[4]; 
					
#define SCL_H         GPIOB->BSRR = GPIO_Pin_6
#define SCL_L         GPIOB->BRR  = GPIO_Pin_6 
   
#define SDA_H         GPIOB->BSRR = GPIO_Pin_7
#define SDA_L         GPIOB->BRR  = GPIO_Pin_7

#define SCL_read      GPIOB->IDR  & GPIO_Pin_6
#define SDA_read      GPIOB->IDR  & GPIO_Pin_7

/* ???? -----------------------------------------------*/
void I2C_GPIO_Config(void);
void Init_MPU9250(void);
void READ_MPU9250_ACCEL(void);
void DATA_printf(u8 *s,short temp_data);
 void Send_data(u8 MAG,u8 axis);
 void READ_MPU9250_GYRO(void);
 void READ_MPU9250_MAG(void);
 void  USART1_SendData(u8 SendData);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void USART1_Configuration(void);
void WWDG_Configuration(void);
void Delay(u32 nTime);
void Delayms(vu32 m);  
#endif
