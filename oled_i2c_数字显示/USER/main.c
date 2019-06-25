#include "led.h"
#include "delay.h"
#include "sys.h"
#include "oled.h"
#include "timer.h"
//ALIENTEK miniSTM32开发板实验1
//跑马灯实验  
//技术支持：www.openedv.com
//广州市星翼电子科技有限公司
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
	delay_init();	    	 //延时函数初始化	  
	LED_Init();		  	//初始化与LED连接的硬件接口
	TIM5_Int_Init(99,7199);		//72000 000  /7200=10000 ,0.1ms  * 100=10ms
	I2C_Configuration();
	OLED_Init();
	OLED_Fill(0x00);//全屏灭
	while(1)
	{
		OLED_Fill(0x00);//全屏灭
		for(i=0;i<5;i++)
		{
			OLED_ShowCN(22+i*16,0,i);//测试显示中文
		}
		delay_ms(1000);
		book=54321;
		fun(book);
		OLED_Fill(0x00);//全屏灭
		OLED_ShowStr(0,3,str,1);//测试6*8字符
		OLED_ShowStr(0,4,"111 Tech",2);				//测试8*16字符
		delay_ms(1000);	
	}
 }

