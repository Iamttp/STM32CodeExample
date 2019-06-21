#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart2.h"
#include "usart.h"
//ALIENTEK miniSTM32开发板实验1
//跑马灯实验  
//技术支持：www.openedv.com
//广州市星翼电子科技有限公司
extern u16 cm2;
 int main(void)
 {	
	 
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	delay_init();	    	 //延时函数初始化	  
	LED_Init();		  	//初始化与LED连接的硬件接口
//	uart_init();
	uart_init(9600);
	uart_init_2();	  
	 LED0=0;			//绿灯亮
	 printf("开始");
	while(1)
	{	
		USART_SendData(USART2,0x55);
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);
		book2=1;
		delay_ms(200);
	}
 }

