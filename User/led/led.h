#ifndef _LED_H
#define _LED_H

#include "stm32f10x.h"


#define LED        PAout(8)
#define LED_ON		 GPIO_SetBits(GPIOB,GPIO_Pin_5)
#define LED_OFF		 GPIO_ResetBits(GPIOB,GPIO_Pin_5)

void LED_GPIO_Config(void);


	
#endif

