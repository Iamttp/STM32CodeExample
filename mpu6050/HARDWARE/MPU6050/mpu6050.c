#include "mpu6050.h"
#include "delay.h"


/**********************************************
*i2c函数   i2c_gpio_init();i2c_start();i2c_stop();
*		   i2c_wait_ack();i2c_ack();i2c_waitack();
*		   IIC_Send_Byte();IIC_Read_Byte();
**********************************************/
//IO方向设置
//IO方向设置
#define SDA_IN()  {GPIOB->CRH&=0XFFFFFFF0;GPIOB->CRH|=8<<0;}
#define SDA_OUT() {GPIOB->CRH&=0XFFFFFFF0;GPIOB->CRH|=3<<0;}

//IO操作函数	 
#define SCL    PBout(9) 		//SCL
#define SDA_w    PBout(8) 		//SDA	 
#define SDA_r  PBin(8) 		//输入SDA 

void i2c_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_TypeDefstruct;
	 GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能PB端口时钟
	GPIO_TypeDefstruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_TypeDefstruct.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9;
	GPIO_TypeDefstruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_TypeDefstruct);
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);//先使能外设IO PORTC时钟 
		
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_11;	 // 端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
  GPIO_Init(GPIOC, &GPIO_InitStructure);					 //根据设定参数初始化GPIO 
	
  GPIO_SetBits(GPIOC,GPIO_Pin_12|GPIO_Pin_11);						 //PB10,PB11 输出高	
 
	SCL=1;
	SDA_w=1;
}

void i2c_start(void)
{
	SDA_OUT();	
	SDA_w=1;
	SCL=1;									//？？？？？？？？？？？？？？？？？？？？？？									////
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

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 i2c_wait_ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
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
	SCL=0;//时钟输出0 	   
	return 0;  
}
//从设备收到数据后会自动返回ACK，判断设备存不存在可以根据ACK信号
//产生ACK应答
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
//不产生ACK应答		    
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
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        SDA_w=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		SCL=1;
		delay_us(2); 
		SCL=0;	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
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
        i2c_nack();//发送nACK
    else
        i2c_ack(); //发送ACK   
    return receive;
}

/****************************************************
*mpu6050函数  MPU_Write_Byte();MPU_Read_Byte();
*			  MPU_Init();
*			  MPU_Set_LPF();MPU_Set_Rate();
****************************************************/
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码

//时序分析： 
//1、开始信号； 
//2、写从设备地址，地址最低位R/W=0，表示为写操作，从设备收到数据后会自动返回ACK，判断设备存不存在可以根据ACK信号； 
//3、写存储地址高八位（写完地址后从设备会返回ACK），(有的设备是10bit的地址，24C64是8bit地址) 
//4、写存储地址低八位（写完地址后从设备会返回ACK） 
//5、写8bit数据(1Byte) 
//6、写停止位
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
    i2c_start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	if(i2c_wait_ack())	//等待应答
	{
		i2c_stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    i2c_wait_ack();		//等待应答 
	IIC_Send_Byte(data);//发送数据
	if(i2c_wait_ack())	//等待ACK
	{
		i2c_stop();	 
		return 1;		 
	}		 
    i2c_stop();	 
	return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据

//时序分析： 
//1、开始信号； 
//2、写从设备地址，地址最低位R/W=1，表示为读操作，从设备收到数据后会自动返回ACK，判断设备存不存在可以根据ACK信号； 
//3、读存储地址高八位（写完地址后从设备会返回ACK），(有的设备是10bit的地址，24C64是8bit地址) 
//4、读存储地址低八位（写完地址后从设备会返回ACK） 
//5、写停止位 
//6、写开始信号 
//7、写从设备地址 
//8、读取数据，并告诉从设备这次不用回ACK 
//9、写停止位			??????
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    i2c_start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	i2c_wait_ack();		//等待应答 
    IIC_Send_Byte(reg);	//写寄存器地址
    i2c_wait_ack();		//等待应答
    i2c_start();
	IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令	
    i2c_wait_ack();		//等待应答 
	res=IIC_Read_Byte(0);//读取数据,发送nACK 
    i2c_stop();			//产生一个停止条件 
	return res;		
}

//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
}

//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}



//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
u8 MPU_Init(void)
{ 
	u8 res; 
	i2c_gpio_init();//初始化IIC总线
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050
    delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
	MPU_Write_Byte(MPU_GYRO_CFG_REG,3<<3);					//陀螺仪传感器,±2000dps
	MPU_Write_Byte(MPU_ACCEL_CFG_REG,0<<3);					//加速度传感器,±2g
	MPU_Set_Rate(50);						//设置采样率50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG); 
	if(res==MPU_ADDR)//器件ID正确
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
		MPU_Set_Rate(50);						//设置采样率为50Hz
 	}else return 1;
	return 0;
}

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	i2c_start(); 
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(i2c_wait_ack())	//等待应答
	{
		i2c_stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    i2c_wait_ack();		//等待应答
    i2c_start();
	IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    i2c_wait_ack();		//等待应答 
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    i2c_stop();	//产生一个停止条件 
	return 0;	
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
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
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
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
//得到温度值
//返回值:温度值(扩大了100倍)
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

