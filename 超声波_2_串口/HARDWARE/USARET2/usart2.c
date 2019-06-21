#include "usart2.h"
#include "usart.h"
#include "delay.h"
u8 Res_h2=0,Res_l2=0;
u8 book2=0;
void uart_init_2(){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStrue;  
    USART_InitTypeDef USART_InitStrue;  
    NVIC_InitTypeDef NVIC_InitStrue;  
      
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);  
    USART_DeInit(USART2);  
      
    // TX-PA2  RX-PA3  
    GPIO_InitStrue.GPIO_Mode=GPIO_Mode_AF_PP;  
    GPIO_InitStrue.GPIO_Pin=GPIO_Pin_2;  
    GPIO_InitStrue.GPIO_Speed=GPIO_Speed_50MHz;  
    GPIO_Init(GPIOA,&GPIO_InitStrue);  
      
    GPIO_InitStrue.GPIO_Mode=GPIO_Mode_IN_FLOATING;  
    GPIO_InitStrue.GPIO_Pin=GPIO_Pin_3;  
  GPIO_Init(GPIOA,&GPIO_InitStrue);  
      
   
    USART_InitStrue.USART_BaudRate=9600; 
    USART_InitStrue.USART_HardwareFlowControl=USART_HardwareFlowControl_None; //无硬件数据流控制
    USART_InitStrue.USART_Mode=USART_Mode_Tx|USART_Mode_Rx; 
    USART_InitStrue.USART_Parity=USART_Parity_No;   
    USART_InitStrue.USART_StopBits=USART_StopBits_1; 
    USART_InitStrue.USART_WordLength=USART_WordLength_8b; 
    USART_Init(USART2,&USART_InitStrue);  
      
    USART_Cmd(USART2,ENABLE);
    USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
      
    NVIC_InitStrue.NVIC_IRQChannel=USART2_IRQn;  
    NVIC_InitStrue.NVIC_IRQChannelCmd=ENABLE;  
    NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority=1;  
    NVIC_InitStrue.NVIC_IRQChannelSubPriority=1;  
    NVIC_Init(&NVIC_InitStrue);
}

u16 cm2;
void USART2_IRQHandler(void)                	//串口2中断服务程序
{
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE))  
	{
		while(USART_GetFlagStatus(USART2,USART_FLAG_RXNE)==RESET);
		if(book2==1)
		{
			Res_h2 =USART_ReceiveData(USART2);	//读取接收到的数据
			book2=0;
		}
		if(book2==0)
		{
			Res_l2=USART_ReceiveData(USART2);	//读取接收到的数据
			cm2=(Res_h2*256+Res_l2)*0.1;  
			printf("%d\n",cm2);
			delay_ms(50);
		}
	}
}
