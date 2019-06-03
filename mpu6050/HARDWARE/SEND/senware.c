#include "sendware.h"

/*!
 *@file			sendware(void *wareaddr, uint32_t waresize);
 *@brief 		����ʾ��������λ������
 *@param		
 *@retval		��
 *@author		TopApex
 *@data			2017-7-29
 *@version		V1.0
*/
void  sendware(void *wareaddr, uint32_t waresize)
{
	#define CMD_WARE     3
    uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    //���ڵ��� ʹ�õ�ǰ����
    uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};    //���ڵ��� ʹ�õĺ�����
	uart_putbuff(cmdf,sizeof(cmdf));
	uart_putbuff((uint8_t *)wareaddr,waresize);
	uart_putbuff(cmdr,sizeof(cmdr));
}

void uart_putbuff(uint8_t *buff,uint32_t len)
{
	while(len--)
	{
		USART_SendData(USART1, *buff);       
		while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);  
		buff++;
	}
}
