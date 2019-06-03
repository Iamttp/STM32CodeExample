#include "led.h"
#include "delay.h"
#include "sys.h"
#include "oled.h"
#include "mpu6050.h"
#include "usart.h"
#include "timer.h"
#include "sendware.h"
#include "math.h"
//ALIENTEK miniSTM32������ʵ��1
//�����ʵ��  
//����֧�֣�www.openedv.com
//������������ӿƼ����޹�˾

unsigned char str0[6]={0};
unsigned char str1[6]={0};
unsigned char str2[6]={0};
unsigned char str3[6]={0};
unsigned char str4[6]={0};
unsigned char str5[6]={0};
short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;	//������ԭʼ����
float gyrox_1,gyroy_1,gyroz_1;
float pitch,roll,yaw; 		//ŷ����
float Accel_X,Accel_Y;
void fun(u16 a,u8 i)
{
	switch (i){
		case 0:
		str0[0]=a%100000/10000+'0';
		str0[1]=a%10000/1000+'0';	//12345 2
		str0[2]=a%1000/100+'0';		//345 3
		str0[3]=a%100/10+'0';		//4
		str0[4]=a%10/1+'0';			//5	
		str0[5]='\0';break;
		case 1:
		str1[0]=a%100000/10000+'0';
		str1[1]=a%10000/1000+'0';	//12345 2
		str1[2]=a%1000/100+'0';		//345 3
		str1[3]=a%100/10+'0';		//4
		str1[4]=a%10/1+'0';			//5	
		str1[5]='\0';break;
		case 2:
		str2[0]=a%100000/10000+'0';
		str2[1]=a%10000/1000+'0';	//12345 2
		str2[2]=a%1000/100+'0';		//345 3
		str2[3]=a%100/10+'0';		//4
		str2[4]=a%10/1+'0';			//5	
		str2[5]='\0';break;
		case 3:
		str3[0]=a%100000/10000+'0';
		str3[1]=a%10000/1000+'0';	//12345 2
		str3[2]=a%1000/100+'0';		//345 3
		str3[3]=a%100/10+'0';		//4
		str3[4]=a%10/1+'0';			//5	
		str3[5]='\0';break;
		case 4:
		str4[0]=a%100000/10000+'0';
		str4[1]=a%10000/1000+'0';	//12345 2
		str4[2]=a%1000/100+'0';		//345 3
		str4[3]=a%100/10+'0';		//4
		str4[4]=a%10/1+'0';			//5	
		str4[5]='\0';break;
		case 5:
		str5[0]=a%100000/10000+'0';
		str5[1]=a%10000/1000+'0';	//12345 2
		str5[2]=a%1000/100+'0';		//345 3
		str5[3]=a%100/10+'0';		//4
		str5[4]=a%10/1+'0';			//5	
		str5[5]='\0';break;
		
	}
}

u16 zhen(short a)
{
	if(a<0)
		return -a;
	else
		return a;
}

int main(void)
{	
	delay_init();	    	 //��ʱ������ʼ��	 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// �����ж����ȼ�����2
	LED_Init();		  	//��ʼ����LED���ӵ�Ӳ���ӿ�
	TIM3_Int_Init(19,7199);//10Khz�ļ���Ƶ�ʣ�������50Ϊ5ms  
	I2C_Configuration();
	OLED_Init();
	MPU_Init();
	
	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
	
	OLED_Fill(0x00);//ȫ����
	delay_ms(10);
	while(1)
	{	
		fun(zhen(Accel_X),0);
		fun(zhen(Accel_Y),1);
		fun(zhen(yaw),2);
//		fun(zhen(gyrox),3);
//		fun(zhen(gyroy),4);
//		fun(zhen(gyroz),5);

		OLED_ShowStr(0,0,str0,2);//����6*8�ַ�
		OLED_ShowStr(0,2,str1,2);				//����8*16�ַ�
		OLED_ShowStr(0,4,str2,2);				//����8*16�ַ�
//		OLED_ShowStr(64,0,str3,2);				//����8*16�ַ�
//		OLED_ShowStr(64,2,str4,2);				//����8*16�ַ�
//		OLED_ShowStr(64,4,str5,2);				//����8*16�ַ�	
		
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
		gyrox_1=gyrox/16.4;  
		gyroy_1=gyroy/16.4;  
		gyroz_1=gyroz/16.4;  
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
		delay_ms(10);
	}
 }


void TIM3_IRQHandler(void)   //TIM3�ж�
{
	
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ 
		/************************************************
		�����˲��������ģ�
		���ݻ��ֽǶ�+=���ٶ�*dt��
		�ںϽǶ�=����Ȩֵ*���ݻ��ֽǶ�+(1-����Ȩֵ)*���ٶȽǶȣ�
		����һ�ֽ�����ݶȷ���
		Angle = (0.98) * (Angle + gyro * dt) + (0.02) *(acc);
		Accel_Y=atan2(Accel_X,Accel_Z)*180/PI;                 //�������	
		Gyro_Y=Gyro_Y/16.4;                                    //����������ת��		
		************************************************/
		
		Accel_X=atan2(aacx,aacz)*57.3; 
		Accel_Y=atan2(aacy,aacz)*57.3;  
		if(aacz<100)
		{
			if(aacx>12000)
				Accel_Y=0;
			if(aacy>12000)
				Accel_X=0;
				
		}		//aacxʱ
		pitch=((Accel_Y)*0.02+(pitch-gyrox_1*0.002)*0.98);
		roll=((Accel_X)*0.02+(roll-gyroy_1*0.002)*0.98);	
//		yaw=(aacz)*0.02+(yaw+gyroz*0.002)*0.98;
		yaw=yaw-gyroz_1*0.002;
		/************************************************
		�����˲�
		************************************************/
	}
}


