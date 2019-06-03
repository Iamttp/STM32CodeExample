#include "timer.h"
#include "led.h"
#include "mpu9250.h"
#include "usart.h"
#include <math.h>
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//通用定时器 驱动代码			   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/12/03
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  
 

//通用定时器中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(  //使能或者失能指定的TIM中断
		TIM3, //TIM2
		TIM_IT_Update ,
		ENABLE  //使能
		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设
							 
}






float angle,angle_m,gyro_m;


void TIM3_IRQHandler(void)   //TIM3中断
{
	static u16 i=0;
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源 
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


