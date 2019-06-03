#ifndef	_SENDWARE_H_
#define _SENDWARE_H_

#include "stm32f10x.h"

void sendware(void *wareaddr,uint32_t waresize);
void uart_putbuff(uint8_t *buff,uint32_t len);

#endif
