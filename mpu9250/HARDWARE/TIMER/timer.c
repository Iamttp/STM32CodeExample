#include "timer.h"
#include "led.h"
#include "mpu9250.h"
#include "usart.h"
#include <math.h>
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//ͨ�ö�ʱ�� ��������			   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2010/12/03
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  
 

//ͨ�ö�ʱ���жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
		TIM3, //TIM2
		TIM_IT_Update ,
		ENABLE  //ʹ��
		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx����
							 
}






float angle,angle_m,gyro_m;


void TIM3_IRQHandler(void)   //TIM3�ж�
{
	static u16 i=0;
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ 
		READ_MPU9250_ACCEL();  //???
//		DATA_printf(TX_DATA,T_X);//??X??????
//		Send_data('A','X');			 //??X?? 
//		DATA_printf(TX_DATA,T_Y);//??Y??????
//		Send_data('A','Y');			 //??Y??
//		DATA_printf(TX_DATA,T_Z);//??Z??????
//		Send_data('A','Z');			 //??Z??
//		
		angle_m=T_Z;
		
		READ_MPU9250_GYRO();      //??
//		DATA_printf(TX_DATA,T_X);//??X??????
//		Send_data('G','X');			 //??X??
//		DATA_printf(TX_DATA,T_Y);//??Y??????
//		Send_data('G','Y');			 //??Y??
//		DATA_printf(TX_DATA,T_Z);//??Z??????
//		Send_data('G','Z');			 //??Z??
		
		 gyro_m=T_Z;
		 
		angle = 0.1 * angle_m+ 0.9 * (angle + gyro_m * 0.005);
		if(i==100)
		{	
			READ_MPU9250_ACCEL();
			DATA_printf(TX_DATA,T_Z);//??Z??????
			Send_data('A','Z');		
			
			DATA_printf(TX_DATA,angle);//??X??????
			Send_data('G','Z');	
			USART1_SendData(0X0D);	 //??
			USART1_SendData(0X0A);	 //??
			i=0;
		}
//		READ_MPU9250_MAG();	      //??
//		DATA_printf(TX_DATA,T_X);//??X??????
//		Send_data('M','X');			 //??X??
//		DATA_printf(TX_DATA,T_Y);//??Y??????
//		Send_data('M','Y');			 //??Y??
//		DATA_printf(TX_DATA,T_Z);//??Z??????
//		Send_data('M','Z');			 //??Z??
		/*
		DATA_printf(TX_DATA,T_T);//?????????
		Send_data('T');			 //??????
		*/
		
		i++;
	}
}


