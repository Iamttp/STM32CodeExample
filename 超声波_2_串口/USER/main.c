#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart2.h"
#include "usart.h"
//ALIENTEK miniSTM32������ʵ��1
//�����ʵ��  
//����֧�֣�www.openedv.com
//������������ӿƼ����޹�˾
extern u16 cm2;
 int main(void)
 {	
	 
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// �����ж����ȼ�����2
	delay_init();	    	 //��ʱ������ʼ��	  
	LED_Init();		  	//��ʼ����LED���ӵ�Ӳ���ӿ�
//	uart_init();
	uart_init(9600);
	uart_init_2();	  
	 LED0=0;			//�̵���
	 printf("��ʼ");
	while(1)
	{	
		USART_SendData(USART2,0x55);
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);
		book2=1;
		delay_ms(200);
	}
 }

