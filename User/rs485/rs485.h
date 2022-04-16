#ifndef _RS485_H
#define _RS485_H

#include "stm32f10x.h"


void RS485_Config(uint32_t bound);
void Usart_Send_Array(uint8_t *array, uint8_t len);

#endif

