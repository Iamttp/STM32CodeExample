#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include  <math.h>    //Keil library  
#include "mpu9250.h"
#include "timer.h"

int main(void)
{ 
	delay_init();
  RCC_Configuration();		 //??RCC
  GPIO_Configuration();		 //??GPIO
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// �����ж����ȼ�����2
  LED_Init();		  	//��ʼ����LED���ӵ�Ӳ���ӿ�
  TIM3_Int_Init(49,7199);//10Khz�ļ���Ƶ�ʣ�������50Ϊ5ms  
  uart_init(9600);
  I2C_GPIO_Config();		 //??IIC????
  Delayms(10);				 //??
  Init_MPU9250();		     //???MPU9250
  printf("��ʼ");
  while(1)
 {
	 
  }
}

/*************??***************/
