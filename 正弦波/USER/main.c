#include "led.h"
#include "delay.h"
#include "sys.h"
#include "exit.h"
#include "usart1.h"
#include "sendware.h"
#include "math.h"
//ALIENTEK Mini STM32�����巶������2
//��������ʵ��		   
//����֧�֣�www.openedv.com
//������������ӿƼ����޹�˾
/******************************
����ʾ���� �� ��ʾ���Ҳ�
******************************/
u32 EncoderValue;
u32 EncoderValue_2;
 int main(void)
 {	
	u32 arr[2]={0};
	delay_init();	    	 //��ʱ������ʼ��	  
	LED_Init();		  	 	//��ʼ����LED���ӵ�Ӳ���ӿ�
	EXTI_init();
	uart1_init(9600);
	LED0=0;					//����LED
	while(1)
	{
		arr[0]=EncoderValue_2;   
		arr[1]=EncoderValue;		   
		sendware((uint8_t*)arr,sizeof(arr));
//		LED0=!LED0;
//		switch(book)
//		{				 
//			case 0:		
//				delay_ms(200);	
//				break;
//			case 1:
//				delay_ms(500);	
//				break;
//			case 2:				
//				delay_ms(1000);	
//				break;
//			default:
//				delay_ms(10);	
//		} 
		EncoderValue++;
		EncoderValue_2=sin(EncoderValue*0.1)*1000+5000;
		delay_ms(50);
	}		 
}
