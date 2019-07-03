/**
  ******************************************************************************
  * @author  ��ҫ�Ƽ� ASHINING
  * @version V3.0
  * @date    2016-10-08
  * @brief   UART����C�ļ�
  ******************************************************************************
  * @attention
  *
  * ����	:	http://www.ashining.com
  * �Ա�	:	https://shop105912646.taobao.com
  * ����Ͱ�:	https://cdzeyao.1688.com
  ******************************************************************************
  */


#include "uart.h"


/**
  * @brief :���ڳ�ʼ��
  * @param :
  *			@UartBaudRate:���ڲ�����
  * @note  :��
  * @retval:��
  */
void drv_uart_init( uint32_t UartBaudRate )
{
	GPIO_InitTypeDef	UartGpioInitStructer;
	USART_InitTypeDef	UartinitStructer;
	
	//�����ù����У�Ϊ��ֹTX RX����ͬһ���˿��ϣ���ǿ����ֲ�ԣ��̷ֿ�����
	//��ʼ������TX RX ���� 
	RCC_APB2PeriphClockCmd( UART_TX_GPIO_CLK | UART_RX_GPIO_CLK, ENABLE );	//��TX RX �˿�ʱ��
	
	UartGpioInitStructer.GPIO_Mode = GPIO_Mode_AF_PP;
	UartGpioInitStructer.GPIO_Speed = GPIO_Speed_2MHz;
	//TX
	UartGpioInitStructer.GPIO_Pin = UART_TX_GPIO_PIN;
	GPIO_Init( UART_TX_GPIO_PORT, &UartGpioInitStructer );		//��ʼ��TX����  ����Ϊ���ù���
	//RX
	UartGpioInitStructer.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	UartGpioInitStructer.GPIO_Pin = UART_RX_GPIO_PIN;
	GPIO_Init( UART_RX_GPIO_PORT, &UartGpioInitStructer );		//��ʼ��RX����  ����Ϊ����
	
	//����USART����
	USART_DeInit( UART_PORT );		//���踴λ
	
	if( USART1 == UART_PORT )		//ʹ������ʱ��
	{
		RCC_APB2PeriphClockCmd( UART_PORT_CLK, ENABLE );			
	}																	//��ͬ��USART��������ڲ�ͬ��APBʱ����														
	else																//STM32F103��Ƭ��ֻ��USART1��APB2�ϣ���ƽ̨�в�������Ӧ�ı伴��
	{
		RCC_APB1PeriphClockCmd( UART_PORT_CLK, ENABLE );
	}
	
	UartinitStructer.USART_BaudRate = UartBaudRate;						//���ò�����
	UartinitStructer.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//��ʹ��������
	UartinitStructer.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;		//���ͺͽ���	
	UartinitStructer.USART_Parity = USART_Parity_No;					//����У��
	UartinitStructer.USART_StopBits = USART_StopBits_1;					//һ��ֹͣλ
	UartinitStructer.USART_WordLength = USART_WordLength_8b;			//8������λ
	
	USART_Cmd( UART_PORT, DISABLE );									//ʧ������
	USART_Init( UART_PORT, &UartinitStructer );							//��ʼ������
	USART_Cmd( UART_PORT, ENABLE );										//ʹ������	
}

/**
  * @brief :���ڷ�������
  * @param :
  *			@TxBuffer:���������׵�ַ
  *			@Length:���ݳ���
  * @note  :��
  * @retval:��
  */
void drv_uart_tx_bytes( uint8_t* TxBuffer, uint8_t Length )
{
	while( Length-- )
	{
		while( RESET == USART_GetFlagStatus( UART_PORT, USART_FLAG_TXE ));
		UART_PORT->DR = *TxBuffer;
		TxBuffer++;
	}
}

/**
  * @brief :���ڽ�������
  * @param :
  *			@RxBuffer:���������׵�ַ
  * @note  :��
  * @retval:���յ����ֽڸ���
  */
uint8_t drv_uart_rx_bytes( uint8_t* RxBuffer )
{
	uint8_t l_RxLength = 0;
	uint16_t l_UartRxTimOut = 0x7FFF;
	
	while( l_UartRxTimOut-- )			//�ȴ���ѯ��������
	{
		if( RESET != USART_GetFlagStatus( UART_PORT, USART_FLAG_RXNE ))
		{
			*RxBuffer = (uint8_t)UART_PORT->DR;
			RxBuffer++;
			l_RxLength++;
			l_UartRxTimOut = 0x7FFF;	//���յ�һ���ַ����ظ��ȴ�ʱ��
		}
		if( 100 == l_RxLength )
		{
			break;		//���ܳ���100���ֽ�
		}
	}
	
	return l_RxLength;					//�ȴ���ʱ�����ݽ������
}

