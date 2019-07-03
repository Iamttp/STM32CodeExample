/**
  ******************************************************************************
  * @author  ��ҫ�Ƽ� ASHINING
  * @version V3.0
  * @date    2016-10-08
  * @brief   ������
  ******************************************************************************
  * @attention
  *
  * ����	:	http://www.ashining.com
  * �Ա�	:	https://shop105912646.taobao.com
  * ����Ͱ�:	https://cdzeyao.1688.com
  ******************************************************************************
  */
 
#include "led.h"
#include "delay.h"
#include "sys.h"
#include "uart.h" 

//#define __AS62_TX_MODE__									/**@@ ����ģʽ�� ���μ�Ϊ����ģʽ @@ **/


#ifdef __AS62_TX_MODE__
	char *pAshining = "ashining";
#else
    uint8_t g_ashining[ 8 ] ={ 'a', 's', 'h', 'i', 'n', 'i', 'n', 'g' };
	uint8_t g_As62_rx_buffer[ 100 ] = { 0 };
	uint8_t g_RxLength = 0;
#endif



/**
  * @brief :������
  * @param :��
  * @note  :��
  * @retval:��
  */
int main( void )
{
	#ifndef __AS62_TX_MODE__
		uint8_t i = 0;
	#endif
	
	//���ڳ�ʼ��
	drv_uart_init( 9600 );
	delay_init();	    	 //��ʱ������ʼ��	  
	LED_Init();		  	//��ʼ����LED���ӵ�Ӳ���ӿ�
	PAout(8)=0; 
	while( 1 )
	{
		//����ģʽ
		#ifdef __AS62_TX_MODE__
			drv_uart_tx_bytes(( uint8_t *)pAshining, 8 );			//���͹̶��ַ��� ashining �ַ���
			delay_ms(1500);
		
		//����ģʽ		
		#else
			g_RxLength = drv_uart_rx_bytes( g_As62_rx_buffer );
			if( 0 != g_RxLength )
			{
				for( i = 0; i < 8; i++ )
				{
					if( g_As62_rx_buffer[ i ] != g_ashining[ i ] )		//�ȶԽ��յ����ַ�
					{
						break;
					}
				}
				if( 8 == i )			//���յ����ֽ����� ashining �ַ�����ͷ��LED��˸һ��
				{
					PAout(8)=1;
				}
			}
		#endif
	}
}


