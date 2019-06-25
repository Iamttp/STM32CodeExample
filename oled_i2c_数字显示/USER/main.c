#include "led.h"
#include "delay.h"
#include "sys.h"
#include "oled.h"
#include "timer.h"
//ALIENTEK miniSTM32������ʵ��1
//�����ʵ��  
//����֧�֣�www.openedv.com
//������������ӿƼ����޹�˾
unsigned char str[6]={0};

void fun(u16 a)
{
	str[0]=a%100000/10000+'0';
	str[1]=a%10000/1000+'0';	//12345 2
	str[2]=a%1000/100+'0';		//345 3
	str[3]=a%100/10+'0';		//4
	str[4]=a%10/1+'0';			//5	
	str[5]='\0';
}

 int main(void)
 {	
	unsigned char i;
	delay_init();	    	 //��ʱ������ʼ��	  
	LED_Init();		  	//��ʼ����LED���ӵ�Ӳ���ӿ�
	TIM5_Int_Init(99,7199);		//72000 000  /7200=10000 ,0.1ms  * 100=10ms
	I2C_Configuration();
	OLED_Init();
	OLED_Fill(0x00);//ȫ����
	while(1)
	{
		OLED_Fill(0x00);//ȫ����
		for(i=0;i<5;i++)
		{
			OLED_ShowCN(22+i*16,0,i);//������ʾ����
		}
		delay_ms(1000);
		book=54321;
		fun(book);
		OLED_Fill(0x00);//ȫ����
		OLED_ShowStr(0,3,str,1);//����6*8�ַ�
		OLED_ShowStr(0,4,"111 Tech",2);				//����8*16�ַ�
		delay_ms(1000);	
	}
 }

