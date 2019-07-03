/**
  ******************************************************************************
  * @author  泽耀科技 ASHINING
  * @version V3.0
  * @date    2016-10-08
  * @brief   主函数
  ******************************************************************************
  * @attention
  *
  * 官网	:	http://www.ashining.com
  * 淘宝	:	https://shop105912646.taobao.com
  * 阿里巴巴:	https://cdzeyao.1688.com
  ******************************************************************************
  */
 
#include "led.h"
#include "delay.h"
#include "sys.h"
#include "uart.h" 

//#define __AS62_TX_MODE__									/**@@ 发送模式， 屏蔽即为接收模式 @@ **/


#ifdef __AS62_TX_MODE__
	char *pAshining = "ashining";
#else
    uint8_t g_ashining[ 8 ] ={ 'a', 's', 'h', 'i', 'n', 'i', 'n', 'g' };
	uint8_t g_As62_rx_buffer[ 100 ] = { 0 };
	uint8_t g_RxLength = 0;
#endif



/**
  * @brief :主函数
  * @param :无
  * @note  :无
  * @retval:无
  */
int main( void )
{
	#ifndef __AS62_TX_MODE__
		uint8_t i = 0;
	#endif
	
	//串口初始化
	drv_uart_init( 9600 );
	delay_init();	    	 //延时函数初始化	  
	LED_Init();		  	//初始化与LED连接的硬件接口
	PAout(8)=0; 
	while( 1 )
	{
		//发送模式
		#ifdef __AS62_TX_MODE__
			drv_uart_tx_bytes(( uint8_t *)pAshining, 8 );			//发送固定字符串 ashining 字符串
			delay_ms(1500);
		
		//接收模式		
		#else
			g_RxLength = drv_uart_rx_bytes( g_As62_rx_buffer );
			if( 0 != g_RxLength )
			{
				for( i = 0; i < 8; i++ )
				{
					if( g_As62_rx_buffer[ i ] != g_ashining[ i ] )		//比对接收到的字符
					{
						break;
					}
				}
				if( 8 == i )			//接收到的字节是以 ashining 字符串开头则LED闪烁一次
				{
					PAout(8)=1;
				}
			}
		#endif
	}
}


