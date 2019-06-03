#include "sendware.h"

/*!
 *@file			sendware(void *wareaddr, uint32_t waresize);
 *@brief 		虚拟示波器的上位机程序
 *@param		
 *@retval		无
 *@author		TopApex
 *@data			2017-7-29
 *@version		V1.0
*/
void  sendware(void *wareaddr, uint32_t waresize)
{
	#define CMD_WARE     3
    uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    //串口调试 使用的前命令
    uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};    //串口调试 使用的后命令
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
