#include "led.h"
#include "delay.h"
#include "sys.h"
#include "exit.h"
#include "usart1.h"
#include "sendware.h"
#include "math.h"
//ALIENTEK Mini STM32开发板范例代码2
//按键输入实验		   
//技术支持：www.openedv.com
//广州市星翼电子科技有限公司
/******************************
虚拟示波器 ， 显示正弦波
******************************/
u32 EncoderValue;
u32 EncoderValue_2;
 int main(void)
 {	
	u32 arr[2]={0};
	delay_init();	    	 //延时函数初始化	  
	LED_Init();		  	 	//初始化与LED连接的硬件接口
	EXTI_init();
	uart1_init(9600);
	LED0=0;					//点亮LED
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
